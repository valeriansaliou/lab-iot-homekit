// Air Conditioner (Remote)
//
// Air conditioner remote controller
// Copyright: 2022, Valerian Saliou <valerian@valeriansaliou.name>
// License: Mozilla Public License v2.0 (MPL v2.0)

const int POLL_EVERY_MILLISECONDS = 30000; // 30 seconds
const int SM_CONVERGE_EVERY_MILLISECONDS = 500; // 1/2 seconds
const int SM_WAKE_UP_EVERY_MILLISECONDS = 2000; // 2 seconds

const unsigned int RANGE_TEMPERATURE_CURRENT_MINIMUM = 0; // 0°C
const unsigned int RANGE_TEMPERATURE_CURRENT_MAXIMUM = 50; // 50°C
const unsigned int RANGE_TEMPERATURE_CURRENT_STEP = 1.0;

const unsigned int RANGE_TEMPERATURE_COOL_MINIMUM = 18; // 18°C
const unsigned int RANGE_TEMPERATURE_COOL_MAXIMUM = 32; // 32°C
const unsigned int RANGE_TEMPERATURE_COOL_STEP = 1.0;

const unsigned int RANGE_TEMPERATURE_HEAT_MINIMUM = 13; // 13°C
const unsigned int RANGE_TEMPERATURE_HEAT_MAXIMUM = 27; // 27°C
const unsigned int RANGE_TEMPERATURE_HEAT_STEP = 1.0;

enum VALUES_ACTIVE {
  ACTIVE_INACTIVE = 0,
  ACTIVE_ACTIVE   = 1
};

enum VALUES_CURRENT_HEATER_COOLER_STATE {
  CURRENT_HEATER_COOLER_STATE_INACTIVE = 0,
  CURRENT_HEATER_COOLER_STATE_IDLE     = 1,
  CURRENT_HEATER_COOLER_STATE_HEATING  = 2,
  CURRENT_HEATER_COOLER_STATE_COOLING  = 3
};

enum VALUES_TARGET_HEATER_COOLER_STATE {
  TARGET_HEATER_COOLER_STATE_OFF  = 0,
  TARGET_HEATER_COOLER_STATE_HEAT = 1,
  TARGET_HEATER_COOLER_STATE_COOL = 2,
  TARGET_HEATER_COOLER_STATE_AUTO = 3
};

enum VALUES_SWING_MODE {
  ACTIVE_SWING_MODE_DISABLED = 0,
  ACTIVE_SWING_MODE_ENABLED  = 1
};

int STATES_DIRECTION_ACTIVE[] = {ACTIVE_INACTIVE, ACTIVE_ACTIVE};

int STATES_DIRECTION_TARGET_HEATER_COOLER_STATE[] = {
  TARGET_HEATER_COOLER_STATE_COOL,
  TARGET_HEATER_COOLER_STATE_HEAT,
  -1, // Fan mode (unsupported)
  TARGET_HEATER_COOLER_STATE_AUTO
};

int STATES_SWING_MODE[] = {
  ACTIVE_SWING_MODE_DISABLED,
  ACTIVE_SWING_MODE_ENABLED
};

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
               delayLoopSMMillis = SM_CONVERGE_EVERY_MILLISECONDS;

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

    // Initialize the state machine values
    initializeStateMachineValues();

    // Initialize HomeKit values (from initial SM values)
    initializeHomeKitValues();
  }

  void loop() {
    // Warning: never block this main loop with a delay(), as this will cause \
    //   the accessory from being marked as 'not responding' on the Home app.
    unsigned int nowMillis = millis();

    // Run poll tasks?
    if (nowMillis - lastLoopPollMillis >= POLL_EVERY_MILLISECONDS) {
      LOG1("[Service:AirConditionerRemote] (poll) Tick in progress...\n");

      // Tick a poll task
      tickTaskPoll();

      LOG1("[Service:AirConditionerRemote] (poll) Tick done, next in %dms\n", POLL_EVERY_MILLISECONDS);

      // Mark last poll time
      lastLoopPollMillis = nowMillis;
    }

    // Tick state machine?
    // Notice: the SM is adaptative, meaning that it can wake up and enter \
    //   into a 'converging mode', and then go to sleep once it has converged \
    //   to the desired configured value. This effectively acts as a debounce, \
    //   as the user may change the value multiple times before settling on \
    //   the final desired value.
    if (nowMillis - lastLoopSMMillis >= delayLoopSMMillis) {
      LOG1("[Service:AirConditionerRemote] (sm) Tick in progress...\n");

      // TODO: IMPORTANT, when receiving a value from HK from the user, then \
      //   bump lastLoopSMMillis to millis() so that the SM delays itself by \
      //   the max debounce value!

      // Tick a state machine task
      // Update next delay loop (still converging, or can go to sleep)
      delayLoopSMMillis = (tickTaskSM() == true) ? SM_WAKE_UP_EVERY_MILLISECONDS : SM_CONVERGE_EVERY_MILLISECONDS;

      LOG1("[Service:AirConditionerRemote] (sm) Tick done, next in %dms\n", delayLoopSMMillis);

      // Mark last tick time
      lastLoopSMMillis = nowMillis;
    }
  }

  void initializeStateMachineValues() {
    // TODO: load all values from the ROM
    smActive = 0;
    smTargetHeaterCoolerState = 0;
    smCoolingThresholdTemperature = 20;
    smHeatingThresholdTemperature = 16;
    smSwingMode = 1;
  }

  void initializeHomeKitValues() {
    forceHomeKitValuesFromStateMachine();
  }

  void forceHomeKitValuesFromStateMachine() {
    // Update with values from the SM
    hkActive->setVal(smActive);
    hkCurrentHeaterCoolerState->setVal(0); // TODO: from smTargetHeaterCoolerState
    hkTargetHeaterCoolerState->setVal(smTargetHeaterCoolerState);
    hkCoolingThresholdTemperature->setVal(smCoolingThresholdTemperature);
    hkHeatingThresholdTemperature->setVal(smHeatingThresholdTemperature);
    hkSwingMode->setVal(smSwingMode);

    LOG1("[Service:AirConditionerRemote] HomeKit values forced from SM:\n");
    LOG1("  - Active = %d\n", hkActive->getVal());
    LOG1("  - Current Heater Cooler State = %d\n", hkCurrentHeaterCoolerState->getVal());
    LOG1("  - Target Heater Cooler State = %d\n", hkTargetHeaterCoolerState->getVal());
    LOG1("  - Cooling Threshold Temperature = %d°C\n", hkCoolingThresholdTemperature->getVal());
    LOG1("  - Heating Threshold Temperature = %d°C\n", hkHeatingThresholdTemperature->getVal());
    LOG1("  - Swing Mode = %d\n", hkSwingMode->getVal());
  }

  void tickTaskPoll() {
    // Acquire current values
    // TODO: poll AC running / stopped and force update SM + HK if changed

    // TODO: poll temperature sensor and force update HK if changed
    hkCurrentTemperature->setVal(21);
  }

  bool tickTaskSM() {
    // High-priority tasks

    // [HIGH] Priority #1: Converge active mode?
    if (hkActive->getVal() != smActive) {
      LOG1("[Service:AirConditionerRemote] (sm : high) Active +1 (hk=%d / sm=%d)\n", hkActive->getVal(), smActive);

      // TODO: acquire next state in circle
      // TODO: send IR signal

      return false;
    }

    // [HIGH] Priority #2: Converge target mode?
    if (hkTargetHeaterCoolerState->getVal() != smTargetHeaterCoolerState) {
      LOG1("[Service:AirConditionerRemote] (sm : high) Mode +1 (hk=%d / sm=%d)\n", hkTargetHeaterCoolerState->getVal(), smTargetHeaterCoolerState);

      // TODO: also update hkCurrentHeaterCoolerState as we converge

      // TODO: acquire next state in circle
      // TODO: send IR signal

      return false;
    }

    if (smActive == ACTIVE_ACTIVE && smTargetHeaterCoolerState > TARGET_HEATER_COOLER_STATE_OFF) {
      // Medium-priority tasks

      // TODO: what about auto? temperature can be set?!
      //   -> TARGET_HEATER_COOLER_STATE_AUTO

      // [MEDIUM] Priority #1: Converge cooling temperature?
      if (smTargetHeaterCoolerState == TARGET_HEATER_COOLER_STATE_COOL && hkCoolingThresholdTemperature->getVal() != smCoolingThresholdTemperature) {
        LOG1("[Service:AirConditionerRemote] (sm : medium) Cool temperature +1 (hk=%d / sm=%d)\n", hkCoolingThresholdTemperature->getVal(), smCoolingThresholdTemperature);

        // TODO: acquire next state in circle
        // TODO: send IR signal

        return false;
      }

      // [MEDIUM] Priority #2: Converge heating temperature?
      if (smTargetHeaterCoolerState == TARGET_HEATER_COOLER_STATE_HEAT && hkHeatingThresholdTemperature->getVal() != smHeatingThresholdTemperature) {
        LOG1("[Service:AirConditionerRemote] (sm : medium) Heat temperature +1 (hk=%d / sm=%d)\n", hkHeatingThresholdTemperature->getVal(), smHeatingThresholdTemperature);

        // TODO: acquire next state in circle
        // TODO: send IR signal

        return false;
      }

      // Low-priority tasks

      // [LOW] Priority #1: Converge swing mode?
      if (hkSwingMode->getVal() != smSwingMode) {
        LOG1("[Service:AirConditionerRemote] (sm : low) Swing +1 (hk=%d / sm=%d)\n", hkSwingMode->getVal(), smSwingMode);

        // TODO

        return false;
      }
    }

    // Has converged (nothing to do)
    return true;
  }

  int progressNextState(int statesCircle[], unsigned int currentState) {
    // TODO

    return -1;
  }

  void emitInfraRedWord(int word) {
    // TODO
  }
};
