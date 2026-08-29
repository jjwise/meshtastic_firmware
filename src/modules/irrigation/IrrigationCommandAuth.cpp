#include "IrrigationCommandAuth.h"

#include <SHA256.h>

#include <cstring>

namespace {
constexpr size_t k_sha256_block_size = 64;
constexpr size_t k_sha256_digest_size = 32;
constexpr size_t k_command_tag_size = 16;
constexpr size_t k_minimum_hmac_key_length = 32;

void clear_bytes(uint8_t *data, size_t length) {
  volatile uint8_t *cursor = data;
  while (length-- > 0) {
    *cursor++ = 0;
  }
}

void append_length_prefixed(String &output, const String &value) {
  output += String(value.length());
  output += ':';
  output += value;
  output += '|';
}

String canonical_command(const IrrigationCommand &command) {
  String canonical = "v1|";
  append_length_prefixed(canonical, command.command_id);
  append_length_prefixed(canonical, command.tracker_id);
  canonical += command.action == IrrigationCommandAction::SET_VALVE ? "V|" : "I|";
  canonical += String(command.issued_at_utc_ms);
  canonical += '|';
  canonical += String(command.expires_at_utc_ms);
  canonical += '|';
  canonical += command.valve_open ? "1|" : "0|";
  canonical += String(command.duration_sec);
  canonical += '|';
  canonical += String(command.position_interval_sec);
  return canonical;
}

void sha256(const uint8_t *data, size_t length, uint8_t digest[k_sha256_digest_size]) {
  SHA256 hash;
  hash.reset();
  hash.update(data, length);
  hash.finalize(digest, k_sha256_digest_size);
}
}  // namespace

String irrigation_command_signature(const IrrigationCommand &command, const String &hmac_key) {
  if (hmac_key.length() < k_minimum_hmac_key_length) {
    return "";
  }

  uint8_t key_block[k_sha256_block_size] = {0};
  if (hmac_key.length() > k_sha256_block_size) {
    sha256(reinterpret_cast<const uint8_t *>(hmac_key.c_str()), hmac_key.length(), key_block);
  } else {
    memcpy(key_block, hmac_key.c_str(), hmac_key.length());
  }

  uint8_t inner_pad[k_sha256_block_size];
  uint8_t outer_pad[k_sha256_block_size];
  for (size_t index = 0; index < k_sha256_block_size; ++index) {
    inner_pad[index] = key_block[index] ^ 0x36;
    outer_pad[index] = key_block[index] ^ 0x5c;
  }

  const String canonical = canonical_command(command);
  uint8_t inner_digest[k_sha256_digest_size];
  SHA256 hash;
  hash.reset();
  hash.update(inner_pad, sizeof(inner_pad));
  hash.update(reinterpret_cast<const uint8_t *>(canonical.c_str()), canonical.length());
  hash.finalize(inner_digest, sizeof(inner_digest));

  uint8_t digest[k_sha256_digest_size];
  hash.reset();
  hash.update(outer_pad, sizeof(outer_pad));
  hash.update(inner_digest, sizeof(inner_digest));
  hash.finalize(digest, sizeof(digest));

  static const char hex[] = "0123456789abcdef";
  String signature;
  signature.reserve(k_command_tag_size * 2);
  for (size_t index = 0; index < k_command_tag_size; ++index) {
    signature += hex[digest[index] >> 4];
    signature += hex[digest[index] & 0x0f];
  }

  clear_bytes(key_block, sizeof(key_block));
  clear_bytes(inner_pad, sizeof(inner_pad));
  clear_bytes(outer_pad, sizeof(outer_pad));
  clear_bytes(inner_digest, sizeof(inner_digest));
  clear_bytes(digest, sizeof(digest));
  return signature;
}

bool irrigation_command_signature_is_valid(const IrrigationCommand &command, const String &hmac_key) {
  const String expected = irrigation_command_signature(command, hmac_key);
  if (expected.length() != command.signature.length() || expected.isEmpty()) {
    return false;
  }

  uint8_t difference = 0;
  for (size_t index = 0; index < expected.length(); ++index) {
    difference |= static_cast<uint8_t>(expected[index] ^ command.signature[index]);
  }
  return difference == 0;
}
