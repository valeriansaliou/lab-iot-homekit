// Air Conditioner (Remote)
//
// Air conditioner remote controller
// Copyright: 2022, Valerian Saliou <valerian@valeriansaliou.name>
// License: Mozilla Public License v2.0 (MPL v2.0)

#include "IRremote.h"
#include "DHT.h"
#include "EEPROM.h"

const int EEPROM_SIZE = 5;
const int EEPROM_ADDRESS_SM_ACTIVE = 0;
const int EEPROM_ADDRESS_SM_TARGET_HEATER_COOLER_STATE = 1;
const int EEPROM_ADDRESS_SM_COOLING_THRESHOLD_TEMPERATURE = 2;
const int EEPROM_ADDRESS_SM_HEATING_THRESHOLD_TEMPERATURE = 3;
const int EEPROM_ADDRESS_SM_SWING_MODE = 4;

const int SENSOR_TEMPERATURE_PIN = 23;
const int SENSOR_TEMPERATURE_DHT_TYPE = DHT11;

const int IR_PIN_PWM = 17;
const int IR_ADDRESS = 0x81;
const int IR_COMMAND_SWITCH_POWER = 0x6B;
const int IR_COMMAND_SWITCH_MODE = 0x66;
const int IR_COMMAND_TOGGLE_FAN_SPEED = 0x64;
const int IR_COMMAND_TOGGLE_SWING = 0x67;
const int IR_COMMAND_TOGGLE_TIMER_MODE = 0x69;
const int IR_COMMAND_TEMPERATURE_INCREASE = 0x65;
const int IR_COMMAND_TEMPERATURE_DECREASE = 0x68;

const int INITIALIZE_HOLD_MILLISECONDS = 500; // 1/2 second
const int POLL_EVERY_MILLISECONDS = 30000; // 30 seconds
const int COMMIT_EVERY_MILLISECONDS = 5000; // 5 seconds
const int SM_CONVERGE_EVERY_MILLISECONDS = 100; // 1/10 second
const int SM_WAKE_UP_EVERY_MILLISECONDS = 1000; // 1 second

const float RANGE_TEMPERATURE_CURRENT_MINIMUM = 0.0; // 0.0°C
const float RANGE_TEMPERATURE_CURRENT_MAXIMUM = 99.0; // 99.0°C
const unsigned int RANGE_TEMPERATURE_CURRENT_STEP = 1.0;

const unsigned int RANGE_TEMPERATURE_COOL_MINIMUM = 18; // 18°C
const unsigned int RANGE_TEMPERATURE_COOL_MAXIMUM = 32; // 32°C
const unsigned int RANGE_TEMPERATURE_COOL_STEP = 1.0;

const unsigned int RANGE_TEMPERATURE_HEAT_MINIMUM = 13; // 13°C
const unsigned int RANGE_TEMPERATURE_HEAT_MAXIMUM = 27; // 27°C
const unsigned int RANGE_TEMPERATURE_HEAT_STEP = 1.0;

enum VALUES_ACTIVE {
  // Supported HK modes
  ACTIVE_INACTIVE = 0,
  ACTIVE_ACTIVE   = 1
};

enum VALUES_CURRENT_HEATER_COOLER_STATE {
  // Supported HK modes
  CURRENT_HEATER_COOLER_STATE_INACTIVE = 0,
  CURRENT_HEATER_COOLER_STATE_IDLE     = 1,
  CURRENT_HEATER_COOLER_STATE_HEATING  = 2,
  CURRENT_HEATER_COOLER_STATE_COOLING  = 3
};

enum VALUES_TARGET_HEATER_COOLER_STATE {
  // Supported HK modes
  TARGET_HEATER_COOLER_STATE_AUTO = 0,
  TARGET_HEATER_COOLER_STATE_HEAT = 1,
  TARGET_HEATER_COOLER_STATE_COOL = 2,

  // Unsupported HK modes
  TARGET_HEATER_COOLER_STATE_UNMAPPED_1 = 3,
  TARGET_HEATER_COOLER_STATE_UNMAPPED_2 = 4
};

enum VALUES_SWING_MODE {
  // Supported HK modes
  ACTIVE_SWING_MODE_DISABLED = 0,
  ACTIVE_SWING_MODE_ENABLED  = 1
};

unsigned int STATES_DIRECTION_ACTIVE[] = {
  ACTIVE_INACTIVE, // 'Off' on the AC unit
  ACTIVE_ACTIVE // 'On' on the AC unit
};

unsigned int STATES_DIRECTION_TARGET_HEATER_COOLER_STATE[] = {
  TARGET_HEATER_COOLER_STATE_HEAT, // 'Heat' on the AC unit
  TARGET_HEATER_COOLER_STATE_AUTO, // 'Cool Auto' on the AC unit
  TARGET_HEATER_COOLER_STATE_COOL, // 'Cool' on the AC unit
  TARGET_HEATER_COOLER_STATE_UNMAPPED_1, // 'Dry' on the AC unit
  TARGET_HEATER_COOLER_STATE_UNMAPPED_2 // 'Fan' on the AC unit
};

unsigned int STATES_COOLING_THRESHOLD_TEMPERATURE[] = {
  RANGE_TEMPERATURE_COOL_MINIMUM,
    19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
  RANGE_TEMPERATURE_COOL_MAXIMUM
};

unsigned int STATES_HEATING_THRESHOLD_TEMPERATURE[] = {
  RANGE_TEMPERATURE_HEAT_MINIMUM,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
  RANGE_TEMPERATURE_HEAT_MAXIMUM
};

unsigned int STATES_SWING_MODE[] = {
  ACTIVE_SWING_MODE_DISABLED,
  ACTIVE_SWING_MODE_ENABLED
};

const unsigned int SIZE_DIRECTION_ACTIVE = 2;
const unsigned int SIZE_DIRECTION_TARGET_HEATER_COOLER_STATE = 5;
const unsigned int SIZE_DIRECTION_COOLING_THRESHOLD_TEMPERATURE = 15;
const unsigned int SIZE_DIRECTION_HEATING_THRESHOLD_TEMPERATURE = 15;
const unsigned int SIZE_DIRECTION_SWING_MODE = 2;

const unsigned int DEFAULT_ACTIVE = ACTIVE_INACTIVE;
const unsigned int DEFAULT_TARGET_HEATER_COOLER_STATE = TARGET_HEATER_COOLER_STATE_COOL;
const unsigned int DEFAULT_THRESHOLD_TEMPERATURE = 18;
const unsigned int DEFAULT_SWING_MODE = ACTIVE_SWING_MODE_ENABLED;

DHT dht(SENSOR_TEMPERATURE_PIN, SENSOR_TEMPERATURE_DHT_TYPE);

struct AirConditionerRemote : Service::HeaterCooler {
  /**
    [HeaterCooler Characteristics]

      - Active
        - 0 = "Inactive"
        - 1 = "Active"

      - CurrentTemperature
        - HAP bounds: [0.0; 100.0] <step = 0.1>
        - AC unit bounds: [xx.0; xx.0] <step = 1.0>

      - CurrentHeaterCoolerState
        - 0 "Inactive"
        - 1 "Idle"
        - 2 "Heating"
        - 3 "Cooling"

      - TargetHeaterCoolerState
        - 0 "Off"
        - 1 "Heat"
        - 2 "Cool"
        - 3 "Auto"

      - CoolingThresholdTemperature
        - HAP bounds: [10.0; 35.0] <step = 0.1>
        - AC unit bounds: [xx.0; xx.0] <step = 1.0>

      - HeatingThresholdTemperature
        - HAP bounds: [0.0; 25.0] <step = 0.1>
        - AC unit bounds: [xx.0; xx.0] <step = 1.0>

      - SwingMode
        - 0 "Swing disabled"
        - 1 "Swing enabled"
  **/

  unsigned int lastLoopPollMillis = 0,
               lastLoopSMMillis = 0,
               lastLoopCommitMillis = 0,
               delayLoopSMMillis = SM_CONVERGE_EVERY_MILLISECONDS;

  bool hasUncommitedEEPROMChanges = false;

  // HomeKit values (might be user-modified)
  SpanCharacteristic *hkActive,
                     *hkCurrentTemperature,
                     *hkCurrentHeaterCoolerState,
                     *hkTargetHeaterCoolerState,
                     *hkCoolingThresholdTemperature,
                     *hkHeatingThresholdTemperature,
                     *hkSwingMode;

  // State Machine internal values (source of truth about the AC unit state)
  unsigned int smActive,
               smTargetHeaterCoolerState,
               smCoolingThresholdTemperature,
               smHeatingThresholdTemperature,
               smSwingMode;

  AirConditionerRemote() : Service::HeaterCooler() {
    // Configure all dependencies
    configureEEPROM();
    configureInfraRed();
    configureSensorTemperature();

    // Configure AC unit characteristics
    hkActive = new Characteristic::Active();
    hkCurrentTemperature = new Characteristic::CurrentTemperature();
    hkCurrentHeaterCoolerState = new Characteristic::CurrentHeaterCoolerState();
    hkTargetHeaterCoolerState = new Characteristic::TargetHeaterCoolerState();
    hkCoolingThresholdTemperature = new Characteristic::CoolingThresholdTemperature();
    hkHeatingThresholdTemperature = new Characteristic::HeatingThresholdTemperature();
    hkSwingMode = new Characteristic::SwingMode();

    // Define the range of numbered characteristics
    hkCurrentTemperature->setRange(RANGE_TEMPERATURE_CURRENT_MINIMUM, RANGE_TEMPERATURE_CURRENT_MAXIMUM, RANGE_TEMPERATURE_CURRENT_STEP);
    hkCoolingThresholdTemperature->setRange(RANGE_TEMPERATURE_COOL_MINIMUM, RANGE_TEMPERATURE_COOL_MAXIMUM, RANGE_TEMPERATURE_COOL_STEP);
    hkHeatingThresholdTemperature->setRange(RANGE_TEMPERATURE_HEAT_MINIMUM, RANGE_TEMPERATURE_HEAT_MAXIMUM, RANGE_TEMPERATURE_HEAT_STEP);

    // Initialize the state machine values + HomeKit values (from initial \
    //   SM values)
    initializeStateMachineValues();
    initializeHomeKitValues();

    // Hold for some time before everything is ready (eg. temperature sensor)
    delay(INITIALIZE_HOLD_MILLISECONDS);
  }

  void loop() {
    // Warning: never block this main loop with a delay(), as this will cause \
    //   the accessory from being marked as 'not responding' on the Home app.
    unsigned int nowMillis = millis();

    // Run poll tasks?
    if ((nowMillis - lastLoopPollMillis) >= POLL_EVERY_MILLISECONDS) {
      LOG2("[Service:AirConditionerRemote] (poll) Tick in progress...\n");

      // Tick a poll task
      tickTaskPoll();

      LOG2("[Service:AirConditionerRemote] (poll) Tick done, next in %dms\n", POLL_EVERY_MILLISECONDS);

      // Mark last poll time
      lastLoopPollMillis = nowMillis;
    }

    // Tick state machine?
    // Notice: the SM is adaptative, meaning that it can wake up and enter \
    //   into a 'converging mode', and then go to sleep once it has converged \
    //   to the desired configured value. This effectively acts as a debounce, \
    //   as the user may change the value multiple times before settling on \
    //   the final desired value.
    if ((nowMillis - lastLoopSMMillis) >= delayLoopSMMillis) {
      LOG2("[Service:AirConditionerRemote] (sm) Tick in progress...\n");

      // Tick a state machine task
      // Update next delay loop (still converging, or can go to sleep)
      delayLoopSMMillis = (tickTaskSM() == true) ? SM_WAKE_UP_EVERY_MILLISECONDS : SM_CONVERGE_EVERY_MILLISECONDS;

      LOG2("[Service:AirConditionerRemote] (sm) Tick done, next in %dms\n", delayLoopSMMillis);

      // Mark last tick time
      lastLoopSMMillis = nowMillis;
    }

    // Run commit tasks?
    if ((nowMillis - lastLoopCommitMillis) >= COMMIT_EVERY_MILLISECONDS) {
      LOG2("[Service:AirConditionerRemote] (commit) Tick in progress...\n");

      // Tick a commit task
      tickTaskCommit();

      LOG2("[Service:AirConditionerRemote] (commit) Tick done, next in %dms\n", COMMIT_EVERY_MILLISECONDS);

      // Mark last commit time
      lastLoopCommitMillis = nowMillis;
    }
  }

  bool update() {
    LOG2("[Service:AirConditionerRemote] (update) Requested...\n");

    // Force the SM in a sleep mode, even if it was currently converging \
    //   (debounce user interactions)
    delayLoopSMMillis = SM_WAKE_UP_EVERY_MILLISECONDS;

    // Force the SM to update later on
    lastLoopSMMillis = millis();

    LOG1("[Service:AirConditionerRemote] (update) Complete. SM will soon converge in %dms.\n", delayLoopSMMillis);

    // Show update as successful
    return true;
  }

  void initializeStateMachineValues() {
    // Load all values from the ROM (or use defaults)
    smActive = readEEPROMOrDefault(EEPROM_ADDRESS_SM_ACTIVE, DEFAULT_ACTIVE);
    smTargetHeaterCoolerState = readEEPROMOrDefault(EEPROM_ADDRESS_SM_TARGET_HEATER_COOLER_STATE, DEFAULT_TARGET_HEATER_COOLER_STATE);
    smCoolingThresholdTemperature = readEEPROMOrDefault(EEPROM_ADDRESS_SM_COOLING_THRESHOLD_TEMPERATURE, DEFAULT_THRESHOLD_TEMPERATURE);
    smHeatingThresholdTemperature = readEEPROMOrDefault(EEPROM_ADDRESS_SM_HEATING_THRESHOLD_TEMPERATURE, DEFAULT_THRESHOLD_TEMPERATURE);
    smSwingMode = readEEPROMOrDefault(EEPROM_ADDRESS_SM_SWING_MODE, DEFAULT_SWING_MODE);
  }

  void initializeHomeKitValues() {
    forceHomeKitValuesFromStateMachine();
  }

  void forceHomeKitValuesFromStateMachine() {
    // Update with values from the SM
    hkActive->setVal(smActive);
    hkTargetHeaterCoolerState->setVal(smTargetHeaterCoolerState);
    hkCoolingThresholdTemperature->setVal(smCoolingThresholdTemperature);
    hkHeatingThresholdTemperature->setVal(smHeatingThresholdTemperature);
    hkSwingMode->setVal(smSwingMode);

    // Apply current mode
    int currentMode = convertTargetModeToCurrentMode(hkActive->getVal(), hkTargetHeaterCoolerState->getVal());

    hkCurrentHeaterCoolerState->setVal(currentMode);

    LOG1("[Service:AirConditionerRemote] HomeKit values forced from SM:\n");
    logSnapshotHKValues();
  }

  void tickTaskCommit() {
    // Should commit unsaved EEPROM changes?
    if (hasUncommitedEEPROMChanges == true) {
      hasUncommitedEEPROMChanges = false;

      LOG2("[Service:AirConditionerRemote] (commit) Unsaved EEPROM changes, committing...\n");

      // Proceed saving of EEPROM
      EEPROM.commit();

      LOG1("[Service:AirConditionerRemote] (commit) Saved EEPROM changes\n");
    }
  }

  void tickTaskPoll() {
    // Acquire current values
    float currentTemperature = acquireTemperatureValue();

    if (currentTemperature >= RANGE_TEMPERATURE_CURRENT_MINIMUM && currentTemperature <= RANGE_TEMPERATURE_CURRENT_MAXIMUM) {
      LOG1("[Service:AirConditionerRemote] (poll) Current temperature: %.2f°C\n", currentTemperature);

      // Update temperature in HK
      hkCurrentTemperature->setVal(currentTemperature);
    } else {
      LOG0("[Service:AirConditionerRemote] (poll) Error acquiring temperature! Too high, too low or none. Is the sensor plugged on IO%d? (got value: %.2f)\n", SENSOR_TEMPERATURE_PIN, currentTemperature);
    }

    LOG1("[Service:AirConditionerRemote] (poll) Current HomeKit values are:\n");
    logSnapshotHKValues();
    LOG1("[Service:AirConditionerRemote] (poll) Current state machine values are:\n");
    logSnapshotSMValues();
  }

  bool tickTaskSM() {
    // High-priority tasks

    // [HIGH] Priority #1: Converge active mode?
    if (hkActive->getVal() != smActive) {
      LOG1("[Service:AirConditionerRemote] (sm : high) Active +1 (hk=%d / sm=%d)\n", hkActive->getVal(), smActive);

      // Update state
      smActive = progressNextState(STATES_DIRECTION_ACTIVE, SIZE_DIRECTION_ACTIVE, smActive, 1, true);

      // Save state
      writeEEPROM(EEPROM_ADDRESS_SM_ACTIVE, smActive);

      // Send IR signal
      emitInfraRedWord(IR_COMMAND_SWITCH_POWER);

      return false;
    }

    // [HIGH] Priority #2: Converge target mode?
    if (hkTargetHeaterCoolerState->getVal() != smTargetHeaterCoolerState) {
      LOG1("[Service:AirConditionerRemote] (sm : high) Mode +1 (hk=%d / sm=%d)\n", hkTargetHeaterCoolerState->getVal(), smTargetHeaterCoolerState);

      // Update state
      smTargetHeaterCoolerState = progressNextState(STATES_DIRECTION_TARGET_HEATER_COOLER_STATE, SIZE_DIRECTION_TARGET_HEATER_COOLER_STATE, smTargetHeaterCoolerState, 1, true);

      // Save state
      writeEEPROM(EEPROM_ADDRESS_SM_TARGET_HEATER_COOLER_STATE, smTargetHeaterCoolerState);

      // Send IR signal
      emitInfraRedWord(IR_COMMAND_SWITCH_MODE);

      // Force-update current mode in HK? (converged)
      if (smTargetHeaterCoolerState == hkTargetHeaterCoolerState->getVal()) {
        // Apply current mode
        int currentMode = convertTargetModeToCurrentMode(smActive, smTargetHeaterCoolerState);

        hkCurrentHeaterCoolerState->setVal(currentMode);
      }

      return false;
    }

    if (smActive == ACTIVE_ACTIVE && smTargetHeaterCoolerState > TARGET_HEATER_COOLER_STATE_AUTO) {
      // Medium-priority tasks

      // [MEDIUM] Priority #1: Converge cooling temperature?
      if (smTargetHeaterCoolerState == TARGET_HEATER_COOLER_STATE_COOL && hkCoolingThresholdTemperature->getVal() != smCoolingThresholdTemperature) {
        LOG1("[Service:AirConditionerRemote] (sm : medium) Cool temperature +1 (hk=%d / sm=%d)\n", hkCoolingThresholdTemperature->getVal(), smCoolingThresholdTemperature);

        // Update state
        int coolingIncrement = smCoolingThresholdTemperature < hkCoolingThresholdTemperature->getVal() ? 1 : -1;

        smCoolingThresholdTemperature = progressNextState(STATES_COOLING_THRESHOLD_TEMPERATURE, SIZE_DIRECTION_COOLING_THRESHOLD_TEMPERATURE, smCoolingThresholdTemperature, coolingIncrement, false);

        // Save state
        writeEEPROM(EEPROM_ADDRESS_SM_COOLING_THRESHOLD_TEMPERATURE, smCoolingThresholdTemperature);

        // Send IR signal
        emitInfraRedWord(coolingIncrement > 0 ? IR_COMMAND_TEMPERATURE_INCREASE : IR_COMMAND_TEMPERATURE_DECREASE);

        return false;
      }

      // [MEDIUM] Priority #2: Converge heating temperature?
      if (smTargetHeaterCoolerState == TARGET_HEATER_COOLER_STATE_HEAT && hkHeatingThresholdTemperature->getVal() != smHeatingThresholdTemperature) {
        LOG1("[Service:AirConditionerRemote] (sm : medium) Heat temperature +1 (hk=%d / sm=%d)\n", hkHeatingThresholdTemperature->getVal(), smHeatingThresholdTemperature);

        // Update state
        int heatingIncrement = smHeatingThresholdTemperature < hkHeatingThresholdTemperature->getVal() ? 1 : -1;

        smHeatingThresholdTemperature = progressNextState(STATES_HEATING_THRESHOLD_TEMPERATURE, SIZE_DIRECTION_HEATING_THRESHOLD_TEMPERATURE, smHeatingThresholdTemperature, heatingIncrement, false);

        // Save state
        writeEEPROM(EEPROM_ADDRESS_SM_HEATING_THRESHOLD_TEMPERATURE, smHeatingThresholdTemperature);

        // Send IR signal
        emitInfraRedWord(heatingIncrement > 0 ? IR_COMMAND_TEMPERATURE_INCREASE : IR_COMMAND_TEMPERATURE_DECREASE);

        return false;
      }

      // Low-priority tasks

      // [LOW] Priority #1: Converge swing mode?
      if (hkSwingMode->getVal() != smSwingMode) {
        LOG1("[Service:AirConditionerRemote] (sm : low) Swing +1 (hk=%d / sm=%d)\n", hkSwingMode->getVal(), smSwingMode);

        // Update state
        smSwingMode = progressNextState(STATES_SWING_MODE, SIZE_DIRECTION_SWING_MODE, smSwingMode, 1, true);

        // Save state
        writeEEPROM(EEPROM_ADDRESS_SM_SWING_MODE, smSwingMode);

        // Send IR signal
        emitInfraRedWord(IR_COMMAND_TOGGLE_SWING);

        return false;
      }
    }

    // Has converged (nothing to do)
    return true;
  }

  unsigned int progressNextState(unsigned int statesCircle[], unsigned int statesCircleSize, unsigned int currentState, int increment, bool circle) {
    // Acquire index of current state in array
    int currentStateIndex = -1;

    for (int i = 0; i < statesCircleSize; i++) {
      // Position found?
      if (currentState == statesCircle[i]) {
        currentStateIndex = i;

        break;
      }
    }

    // State not found? This is not expected!
    if (currentStateIndex < 0) {
      LOG0("[Service:AirConditionerRemote] (error) State not found in circle! This is not expected?\n");

      // Fallback to first available value
      return statesCircle[0];
    }

    // Acquire next state (index = n+1 or 0)
    unsigned int nextStateIndex = currentStateIndex + increment;

    if (nextStateIndex >= statesCircleSize) {
      nextStateIndex = (circle == true) ? 0 : (statesCircleSize - 1);
    } else if (nextStateIndex < 0) {
      nextStateIndex = (circle == true) ? (statesCircleSize - 1) : 0;
    }

    return statesCircle[nextStateIndex];
  }

  void configureEEPROM() {
    EEPROM.begin(sizeof(int) * EEPROM_SIZE);
  }

  void configureSensorTemperature() {
    dht.begin();
  }

  void configureInfraRed() {
    IrSender.begin(IR_PIN_PWM);
  }

  float acquireTemperatureValue() {
    // Read temperature on DHT sensor
    float temperature = dht.readTemperature();

    // Invalid temperature acquired?
    if (isnan(temperature) == true) {
      return -1.0;
    }

    // Valid temperature acquired
    return temperature;
  }

  void emitInfraRedWord(int word) {
    IrSender.sendNEC(IR_ADDRESS, word, 1);
  }

  int convertTargetModeToCurrentMode(int active, int targetMode) {
    int currentMode = CURRENT_HEATER_COOLER_STATE_INACTIVE;

    if (active == ACTIVE_ACTIVE) {
      if (targetMode == TARGET_HEATER_COOLER_STATE_COOL || targetMode == TARGET_HEATER_COOLER_STATE_AUTO) {
        currentMode = CURRENT_HEATER_COOLER_STATE_COOLING;
      } else if (targetMode == TARGET_HEATER_COOLER_STATE_HEAT) {
        currentMode = CURRENT_HEATER_COOLER_STATE_HEATING;
      } else {
        currentMode = CURRENT_HEATER_COOLER_STATE_IDLE;
      }
    }

    return currentMode;
  }

  unsigned int readEEPROMOrDefault(int address, unsigned int defaultValue) {
    unsigned int savedValue = EEPROM.read(address);

    // Value empty? (ie. EEPROM is empty)
    if (savedValue == 255) {
      return defaultValue;
    }

    // Value is set (ie. EEPROM has data)
    return savedValue;
  }

  void writeEEPROM(int address, unsigned int value) {
    // Force the SM to update later on + schedule commit
    lastLoopCommitMillis = millis();
    hasUncommitedEEPROMChanges = true;

    // Write new value
    EEPROM.write(address, value);
  }

  void logSnapshotHKValues() {
    LOG1("  - Active = %d\n", hkActive->getVal());
    LOG1("  - Current Heater Cooler State = %d\n", hkCurrentHeaterCoolerState->getVal());
    LOG1("  - Target Heater Cooler State = %d\n", hkTargetHeaterCoolerState->getVal());
    LOG1("  - Cooling Threshold Temperature = %d°C\n", hkCoolingThresholdTemperature->getVal());
    LOG1("  - Heating Threshold Temperature = %d°C\n", hkHeatingThresholdTemperature->getVal());
    LOG1("  - Swing Mode = %d\n", hkSwingMode->getVal());
  }

  void logSnapshotSMValues() {
    LOG1("  - Active = %d\n", smActive);
    LOG1("  - Target Heater Cooler State = %d\n", smTargetHeaterCoolerState);
    LOG1("  - Cooling Threshold Temperature = %d°C\n", smCoolingThresholdTemperature);
    LOG1("  - Heating Threshold Temperature = %d°C\n", smHeatingThresholdTemperature);
    LOG1("  - Swing Mode = %d\n", smSwingMode);
  }
};
