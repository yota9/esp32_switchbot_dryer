#include "BLEDevice.h"

#define BUTTON_PIN 0
#define LED_PIN 2
#define HC_SR04_TRIG_PIN 12
#define HC_SR04_ECHO_PIN 14

// Minimal ms time to wait at start clean/start chage time to ensure robot is in proper state
static const unsigned long MinDeltaWaitTime = 7 * 1000 * 60; // 7 min
// Time before dryer would auto turned off
static const unsigned long MinDeltaDryerTime = 3 * 1000 * 60 * 60; // 3 hours
// Min and max cm distance delta to consider ~10cm robot movement
static const unsigned MinDistanceDelta = 5;
static const unsigned MaxDistanceDelta = 15;

// Device service
static const BLEUUID ServiceUUID("cba20d00-224d-11e6-9fb8-0002a5d5c51b");
// The Notify characteristic
static const BLEUUID CharCmdUUID("cba20002-224d-11e6-9fb8-0002a5d5c51b");
// The TX characteristic
static const BLEUUID CharNotifyUUID("cba20003-224d-11e6-9fb8-0002a5d5c51b");

class NotifyStatus {
  bool WaitingForNotify{false};
  bool Notify{false};
  unsigned CheckAttempts{0};
  static const unsigned MaxAttempts = 5;

public:
  void setWaitingForNotify() {
    CheckAttempts = 0;
    WaitingForNotify = true;
    Notify = false;
  }

  void setNotify(bool Force = false) {
    if (Force || WaitingForNotify) {
      WaitingForNotify = false;
      Notify = true;
    }
  }

  bool checkAndResetNotify() {
    if (Notify) {
      Notify = false;
      return true;
    }

    ++CheckAttempts;
    return false;
  }

  bool hasAttempts() {
    return Notify ? true : WaitingForNotify && CheckAttempts < MaxAttempts;
  }
};

enum NotifyEnum {
  BLUETOOTH_REPLY_NOTIFY = 0,
  BUTTON_PRESSED_NOTIFY,
  NOTIFY_MAX,
};

static NotifyStatus NotifyStatus[NOTIFY_MAX];

enum States {
  WAITING_FOR_ROBOT_START_MOVE_STATE = 0,
  WAITING_FOR_ROBOT_START_CLEAN_STATE,
  WAITING_FOR_ROBOT_CLEAN_FINISH_STATE,
  WAITING_FOR_ROBOT_START_CHARGE_STATE,
  ENABLE_DRYER_STATE,
};

struct Context {
  enum States State{WAITING_FOR_ROBOT_START_MOVE_STATE};
  unsigned long ChangedStateTime{0};
  unsigned long LastPushEnabledTime{0};
  bool DryerState{false};
  unsigned DefaultDistance{0};
};

static BLEAdvertisedDevice *SwitchBotDevice{nullptr};
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(ServiceUUID)) {
      BLEDevice::getScan()->stop();
      SwitchBotDevice = new BLEAdvertisedDevice(advertisedDevice);
    }
  }
};

static void blinkLed(unsigned Times, unsigned Delay = 500) {
  while(Times--) {
    digitalWrite(LED_PIN, HIGH);
    delay(Delay);
    digitalWrite(LED_PIN, LOW);
    delay(Delay);
  }
}

static void updateState(struct Context *Ctx, enum States NewState) {
    Serial.print("Setting new state to ");
    Serial.println(NewState);
    blinkLed((unsigned)NewState + 1);
    Ctx->State = NewState;
    Ctx->ChangedStateTime = millis();  
}

static void deepSleepDelay(unsigned long Seconds) {
  Serial.print("Going to sleep seconds: ");
  Serial.println(Seconds);
  delay(Seconds * 1000);
}

static float measureDistance(const unsigned Times = 1) {
  float Distance = 0;
  for (unsigned T = 0; T < Times; ++T) {
    digitalWrite(HC_SR04_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_SR04_TRIG_PIN, LOW);
    const long Duration = pulseIn(HC_SR04_ECHO_PIN, HIGH);
    Distance += Duration * 0.017;
  }
  return Distance / Times;
}

static void updateDefaultDistance(struct Context *Ctx) {
  Ctx->DefaultDistance = measureDistance(/*Times*/3);
  Serial.print("Updating default distance to ");
  Serial.println(Ctx->DefaultDistance);
}

static int hasDistanceChanged(struct Context *Ctx) {
  const float Distance = measureDistance();
  const int Delta = Distance - Ctx->DefaultDistance;
  if (abs(Delta) < MinDistanceDelta || abs(Delta) > MaxDistanceDelta)
    return 0;
    
  return Delta;
}

static void bluetoothNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                    uint8_t* pData, size_t Length, bool IsNotify) {
  if (Length != 3)
    return;
  
  // Success reply from switchbot
  if (pData[0] == 1 && pData[1] == 0xFF && pData[2] == 0)
    NotifyStatus[BLUETOOTH_REPLY_NOTIFY].setNotify();
}

static int pressSwitchBot() {
  Serial.print("Pressing switchbot... ");
  const uint8_t PushCmd[] = {0x57, 0x01, 0x00};
  const char *Str = "OK";
  int Ret = -1;
  
  BLERemoteService *pRemoteService;
  BLERemoteCharacteristic *pRemoteCharacteristic;
  BLERemoteDescriptor* pDescriptor;

  BLEClient *pClient = BLEDevice::createClient();
  if (!pClient->connect(SwitchBotDevice)) {
    Str = "Failed (no connection)";
    goto out;
  }

  pRemoteService  = pClient->getService(ServiceUUID);
  if (!pRemoteService) {
    Str = "Failed (no service found)";
    goto out;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(CharNotifyUUID);
  if (!pRemoteCharacteristic) {
    Str = "Failed (no TX characteristics)";
    goto out;
  }
  
  NotifyStatus[BLUETOOTH_REPLY_NOTIFY].setWaitingForNotify();
  pRemoteCharacteristic->registerForNotify(bluetoothNotifyCallback);
  delay(50);

  pRemoteCharacteristic = pRemoteService->getCharacteristic(CharCmdUUID);
  if (!pRemoteCharacteristic) {
    Str = "Failed (no RX characteristics)";
    goto out;
  }

  // Push CMD
  pRemoteCharacteristic->writeValue((uint8_t *)PushCmd, sizeof(PushCmd), true); 
  delay(200);
  
  // Wait for notify
  while (!NotifyStatus[BLUETOOTH_REPLY_NOTIFY].checkAndResetNotify()) {
    if (!NotifyStatus[BLUETOOTH_REPLY_NOTIFY].hasAttempts()) {
      Str = "Failed (reply expected)";
      goto out;
    }

    delay(200);
  }
  
  Str = "OK";
  Ret = 0;

out:
  if (pClient->isConnected())
    pClient->disconnect();

  Serial.println(Str);
  return Ret;
}

static int pressSwitchBot(int Attempts) {
  while (--Attempts > 0 && pressSwitchBot() < 0)
    delay(3000);
  
  return Attempts > 0 ? 0 : pressSwitchBot();
}

static int pressSwitchBot(struct Context *Ctx, unsigned Attempts = 5) {
  if (pressSwitchBot(Attempts))
    return -1;
    
  Ctx->DryerState = !Ctx->DryerState;
  if (Ctx->DryerState)
    Ctx->LastPushEnabledTime = millis();

  return 0;
}

static void buttonPress() {
  NotifyStatus[BUTTON_PRESSED_NOTIFY].setNotify(/*Force*/true);
}

static void bleSetup() {
  Serial.print("Searching for switchbot... ");
  BLEDevice::init("ESP32");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true); 

  do {
    const unsigned TimeToScan = 10;
    unsigned long ScanStart = millis();
    pBLEScan->start(TimeToScan, false);
    do {
      if (SwitchBotDevice)
        break;

      delay(100);
    } while(millis() - ScanStart < TimeToScan * 1000);
  } while (!SwitchBotDevice);

  Serial.println("Done!");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HC_SR04_TRIG_PIN, OUTPUT);
  pinMode(HC_SR04_ECHO_PIN, INPUT);

  digitalWrite(HC_SR04_TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPress, RISING);
  bleSetup();
}

void loop() {
  struct Context Ctx[1];
  updateDefaultDistance(Ctx);
  while(1) {
    // Check button was pressed
    if (NotifyStatus[BUTTON_PRESSED_NOTIFY].checkAndResetNotify())
      pressSwitchBot(Ctx);  

    // Update dryer state if it auto turned off due to timeout
    if (Ctx->DryerState && labs(millis() - Ctx->LastPushEnabledTime) > MinDeltaDryerTime)
      Ctx->DryerState = false;

    switch(Ctx->State) {
      case WAITING_FOR_ROBOT_START_MOVE_STATE: {
        int Delta = hasDistanceChanged(Ctx);
        if (Delta > 0) {
          // Robot goes out of the dock
          updateState(Ctx, WAITING_FOR_ROBOT_START_CLEAN_STATE);    
        } else if (Delta < 0) {
          // Robot goes in dock
          // Update DefaultDistance to be robot in-dock value
          updateDefaultDistance(Ctx);
          // Measure again to ensure that the robot is inplace or restart
          if (hasDistanceChanged(Ctx))
            break;
            
          updateState(Ctx, WAITING_FOR_ROBOT_START_CHARGE_STATE);
        } else {
          deepSleepDelay(10); 
        }
      }
      break;

      case WAITING_FOR_ROBOT_START_CLEAN_STATE: {
        if (hasDistanceChanged(Ctx) == 0) {
          updateState(Ctx, WAITING_FOR_ROBOT_START_MOVE_STATE);
          break;
        }

        if (labs(millis() - Ctx->ChangedStateTime) < MinDeltaWaitTime) {
          deepSleepDelay(5);
          break;
        }

        updateState(Ctx, WAITING_FOR_ROBOT_CLEAN_FINISH_STATE);
      }
      break;
    
      case WAITING_FOR_ROBOT_CLEAN_FINISH_STATE: {
        // Disable dryer if enabled
        if (Ctx->DryerState)
          pressSwitchBot(Ctx);

        if (hasDistanceChanged(Ctx) == 0) {
          updateState(Ctx, WAITING_FOR_ROBOT_START_CHARGE_STATE);
          break;
        }

         deepSleepDelay(10);
      }
      break;

      case WAITING_FOR_ROBOT_START_CHARGE_STATE: {
        if (hasDistanceChanged(Ctx) > 0) {
          updateState(Ctx, WAITING_FOR_ROBOT_CLEAN_FINISH_STATE);
          break;
        }

        if (labs(millis() - Ctx->ChangedStateTime) < MinDeltaWaitTime) {
          deepSleepDelay(5);
          break;
        }

        updateState(Ctx, ENABLE_DRYER_STATE);
      }
      break;

      case ENABLE_DRYER_STATE: {
        if (!Ctx->DryerState)
          pressSwitchBot(Ctx);

        updateState(Ctx, WAITING_FOR_ROBOT_START_MOVE_STATE);
      }
      break;
    }
  }
}
