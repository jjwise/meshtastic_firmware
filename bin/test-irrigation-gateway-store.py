#!/usr/bin/env python3
"""Run the real gateway store against a host filesystem double; no device needed."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent

ARDUINO = r"""
#pragma once
#include <algorithm>
#include <cctype>
#include <string>
#include <utility>
class String {
  std::string value_;
  void add() { live += value_.size(); peak = std::max(peak, live); }
  void remove() { live -= value_.size(); }
public:
  static inline size_t live = 0, peak = 0;
  String() = default;
  String(const char *s) : value_(s) { add(); }
  String(std::string s) : value_(std::move(s)) { add(); }
  String(const String &s) : value_(s.value_) { add(); }
  String(String &&s) noexcept {
    s.remove(); value_ = std::move(s.value_); s.value_.clear(); add();
  }
  ~String() { remove(); }
  String &operator=(const String &s) {
    if (this != &s) { remove(); value_ = s.value_; add(); }
    return *this;
  }
  String &operator=(String &&s) noexcept {
    if (this != &s) { remove(); s.remove(); value_ = std::move(s.value_); s.value_.clear(); add(); }
    return *this;
  }
  size_t length() const { return value_.size(); }
  bool isEmpty() const { return value_.empty(); }
  const char *c_str() const { return value_.c_str(); }
  void trim() {
    remove();
    auto first = value_.find_first_not_of(" \t\r\n");
    auto last = value_.find_last_not_of(" \t\r\n");
    value_ = first == std::string::npos ? "" : value_.substr(first, last - first + 1);
    add();
  }
  friend String operator+(const String &a, const char *b) { return String(a.value_ + b); }
};
"""

FILESYSTEM = r"""
#pragma once
#include "Arduino.h"
#include <map>
#include <memory>
#define FILE_READ "r"
#define FILE_WRITE "w"
struct FileState { std::string *contents; size_t pos = 0; bool closed = false; };
class File {
  std::shared_ptr<FileState> state_;
public:
  File() = default;
  explicit File(std::string *contents) : state_(std::make_shared<FileState>(FileState{contents})) {}
  explicit operator bool() const { return state_ && !state_->closed; }
  bool available() const { return *this && state_->pos < state_->contents->size(); }
  String readStringUntil(char delimiter) {
    auto end = state_->contents->find(delimiter, state_->pos);
    if (end == std::string::npos) end = state_->contents->size();
    String line(state_->contents->substr(state_->pos, end - state_->pos));
    state_->pos = std::min(end + 1, state_->contents->size());
    return line;
  }
  size_t println(const String &line);
  void flush() {}
  void close() { if (state_) state_->closed = true; }
};
class FakeFS {
public:
  std::map<std::string, std::string> files;
  bool fail_read = false, fail_open_write = false, fail_rename = false;
  long write_budget = -1;
  bool begin(bool) { return true; }
  bool exists(const String &p) { return files.count(p.c_str()); }
  File open(const String &p, const char *mode) {
    if (std::string(mode) == FILE_READ) {
      if (fail_read || !exists(p)) return {};
    } else {
      if (fail_open_write) return {};
      files[p.c_str()].clear();
    }
    return File(&files[p.c_str()]);
  }
  bool remove(const String &p) { return files.erase(p.c_str()) != 0; }
  bool rename(const String &a, const String &b) {
    if (fail_rename || !exists(a)) return false;
    files[b.c_str()] = std::move(files[a.c_str()]);
    files.erase(a.c_str());
    return true;
  }
};
inline FakeFS LittleFS;
inline size_t File::println(const String &line) {
  std::string bytes = std::string(line.c_str()) + "\r\n";
  size_t count = bytes.size();
  if (LittleFS.write_budget >= 0) {
    count = std::min(count, static_cast<size_t>(LittleFS.write_budget));
    LittleFS.write_budget -= count;
  }
  state_->contents->append(bytes.data(), count);
  return count;
}
"""

TEST = r"""
#include "IrrigationGatewayStore.h"
#include "LittleFS.h"
#include <cassert>
#include <iostream>

int main() {
  IrrigationGatewayStore store;
  assert(store.begin("/queue", 3));
  String first("sentinel");
  assert(store.size() == 0 && !store.peek_next(first) && !store.pop_next());
  assert(std::string(first.c_str()) == "sentinel");
  assert(store.append("one") && store.append("two") && store.append("three"));
  assert(store.size() == 3 && store.peek_next(first) && std::string(first.c_str()) == "one");
  assert(store.append("four"));
  assert(LittleFS.files["/queue"] == "two\r\nthree\r\nfour\r\n");
  assert(store.pop_next() && store.size() == 2);
  assert(store.peek_next(first) && std::string(first.c_str()) == "three");

  LittleFS.files["/queue"] = " \n  alpha \r\n\n beta\n";
  assert(store.size() == 2 && store.peek_next(first) && std::string(first.c_str()) == "alpha");
  assert(store.pop_next() && LittleFS.files["/queue"] == "beta\r\n");

  const std::string saved = LittleFS.files["/queue"];
  LittleFS.fail_open_write = true;
  assert(!store.append("lost") && LittleFS.files["/queue"] == saved);
  LittleFS.fail_open_write = false;
  LittleFS.write_budget = 3;
  assert(!store.append("lost") && LittleFS.files["/queue"] == saved);
  assert(!LittleFS.files.count("/queue.tmp"));
  LittleFS.write_budget = -1;
  LittleFS.fail_rename = true;
  assert(!store.pop_next() && LittleFS.files["/queue"] == saved);
  LittleFS.fail_rename = false;
  LittleFS.fail_read = true;
  assert(!store.append("lost") && !store.peek_next(first) && !store.pop_next());
  assert(LittleFS.files["/queue"] == saved);
  LittleFS.fail_read = false;

  assert(store.begin("/zero", 0) && store.append("discarded") && store.size() == 0);
  assert(store.begin("/one", 1) && store.append("old") && store.append("new"));
  assert(store.size() == 1 && store.peek_next(first) && std::string(first.c_str()) == "new");
  assert(store.pop_next() && store.size() == 0 && !store.pop_next());

  assert(store.begin("/large", 500));
  std::string records;
  for (int i = 0; i < 500; ++i)
    records += std::to_string(i) + ":" + std::string(400, 'x') + "\r\n";
  LittleFS.files["/large"] = records;
  String::peak = String::live;
  assert(store.size() == 500);
  assert(store.peek_next(first) && std::string(first.c_str()).rfind("0:", 0) == 0);
  assert(store.append("tail"));
  assert(store.peek_next(first) && std::string(first.c_str()).rfind("1:", 0) == 0);
  assert(store.pop_next() && store.size() == 499);
  assert(store.peek_next(first) && std::string(first.c_str()).rfind("2:", 0) == 0);
  assert(LittleFS.files["/large"].substr(LittleFS.files["/large"].size() - 6) == "tail\r\n");
  assert(String::peak < 4096);
  std::cout << "PASS: FIFO, capacity, whitespace, empty/zero/one, read/write/rename failures\n"
            << "PASS: 500-record backlog; peak logical String bytes in host double = "
            << String::peak << " (not an ESP32 heap measurement)\n";
}
"""

def main():
    with tempfile.TemporaryDirectory(prefix="irrigation-store-") as temp:
        directory = pathlib.Path(temp)
        for name, contents in {
            "Arduino.h": ARDUINO,
            "LittleFS.h": FILESYSTEM,
            "DebugConfiguration.h": "#pragma once\n#define LOG_WARN(...) ((void)0)\n",
            "test.cpp": TEST,
        }.items():
            (directory / name).write_text(contents)
        binary = directory / "gateway-store-test"
        module = ROOT / "src/modules/irrigation"
        subprocess.run([
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-DARCH_ESP32",
            "-I", str(directory), "-I", str(module),
            str(directory / "test.cpp"), str(module / "IrrigationGatewayStore.cpp"),
            "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    main()
