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

#if defined(IRRIGATION_IMU_TEST) && defined(ARCH_ESP32)
  enum class ImuTestState {
    WAIT_FOR_INITIALIZATION,
    WAIT_FOR_AWAKE_INTERRUPT,
    WAIT_FOR_DEEP_SLEEP,
    COMPLETE
  };

  void runImuTest();
  bool initializeImuTest();
  void handleImuTestInterrupt();
  void enterImuTestDeepSleep();

  ImuTestState imuTestState_;
  uint32_t imuTestStateStartedMs_;
  uint32_t imuTestLastInitAttemptMs_;
#endif

  IrrigationModule irrigation_;
  bool initialized_;
};

extern IrrigationOverlayModule *irrigationOverlayModule;

#endif
