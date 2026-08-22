#pragma once

#ifdef IRRIGATION_MODULE_ENABLE

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "mesh/NodeDB.h"
#include "modules/irrigation/IrrigationConfig.h"
#include "modules/irrigation/IrrigationModule.h"

class IrrigationOverlayModule : public SinglePortModule, private concurrency::OSThread {
 public:
  IrrigationOverlayModule();
  virtual int32_t runOnce() override;

 protected:
  virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

 private:
  void ensureInitialized();
  IrrigationConfig makeRuntimeConfig() const;
  IrrigationSensorSample readSensorSampleFromNodeDb() const;
  uint64_t nowMs() const;
  bool trySendPendingMeshTx();

  IrrigationModule irrigation_;
  bool initialized_;
};

extern IrrigationOverlayModule *irrigationOverlayModule;

#endif
