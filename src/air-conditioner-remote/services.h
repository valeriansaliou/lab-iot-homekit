// Air Conditioner (Remote)
//
// Air conditioner remote controller
// Copyright: 2022, Valerian Saliou <valerian@valeriansaliou.name>
// License: Mozilla Public License v2.0 (MPL v2.0)

const int POLL_EVERY_MILLISECONDS = 30000; // 30 seconds

const unsigned int RANGE_TEMPERATURE_CURRENT_MINIMUM = 0; // 0°C
const unsigned int RANGE_TEMPERATURE_CURRENT_MAXIMUM = 50; // 50°C
const unsigned int RANGE_TEMPERATURE_CURRENT_STEP = 1.0;

const unsigned int RANGE_TEMPERATURE_COOL_MINIMUM = 18; // 18°C
const unsigned int RANGE_TEMPERATURE_COOL_MAXIMUM = 32; // 32°C
const unsigned int RANGE_TEMPERATURE_COOL_STEP = 1.0;

const unsigned int RANGE_TEMPERATURE_HEAT_MINIMUM = 13; // 13°C
const unsigned int RANGE_TEMPERATURE_HEAT_MAXIMUM = 27; // 27°C
const unsigned int RANGE_TEMPERATURE_HEAT_STEP = 1.0;

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

  bool valuesInitialized;

  SpanCharacteristic *active,
                     *currentTemperature,
                     *currentHeaterCoolerState,
                     *targetHeaterCoolerState,
                     *coolingThresholdTemperature,
                     *heatingThresholdTemperature,
                     *swingMode;

  AirConditionerRemote() : Service::HeaterCooler() {
    // TODO: write docs in README

    // Mark values as not initialized
    valuesInitialized = false;

    // Configure AC unit characteristics
    active = new Characteristic::Active();
    currentTemperature = new Characteristic::CurrentTemperature();
    currentHeaterCoolerState = new Characteristic::CurrentHeaterCoolerState();
    targetHeaterCoolerState = new Characteristic::TargetHeaterCoolerState();
    coolingThresholdTemperature = new Characteristic::CoolingThresholdTemperature();
    heatingThresholdTemperature = new Characteristic::HeatingThresholdTemperature();
    swingMode = new Characteristic::SwingMode();

    // Define the range of numbered characteristics
    currentTemperature->setRange(RANGE_TEMPERATURE_CURRENT_MINIMUM, RANGE_TEMPERATURE_CURRENT_MAXIMUM, RANGE_TEMPERATURE_CURRENT_STEP);
    coolingThresholdTemperature->setRange(RANGE_TEMPERATURE_COOL_MINIMUM, RANGE_TEMPERATURE_COOL_MAXIMUM, RANGE_TEMPERATURE_COOL_STEP);
    heatingThresholdTemperature->setRange(RANGE_TEMPERATURE_HEAT_MINIMUM, RANGE_TEMPERATURE_HEAT_MAXIMUM, RANGE_TEMPERATURE_HEAT_STEP);
  }

  void loop() {
    // Wait until next poll is possible
    // Warning: never block this main loop with a delay(), as this will cause \
    //   the accessory from being marked as 'not responding' on the Home app.
    if (valuesInitialized == false || active->timeVal() > POLL_EVERY_MILLISECONDS){
      LOG1("[Service:AirConditionerRemote] Loop tick in progress...\n");
  
      // Check current values from the main logic board
      pollAndUpdate();
  
      LOG1("[Service:AirConditionerRemote] Loop tick done, next in %dms\n", POLL_EVERY_MILLISECONDS);

      // Mark values as initialized (used for the first pass only)
      valuesInitialized = true;
    }
  }

  void pollAndUpdate() {
    // Acquire current values
    // TODO: poll real values from the logic board (no forced value)
    unsigned int tickActive = 1; // Active
    unsigned int tickCurrentTemperature = 21; // 21C
    unsigned int tickCurrentHeaterCoolerState = 3; // Cooling
    unsigned int tickTargetHeaterCoolerState = 2; // Cool
    unsigned int tickCoolingThresholdTemperature = 20; // 20C
    unsigned int tickHeatingThresholdTemperature = 16; // 16C
    unsigned int tickSwingMode = 1; // Enabled

    // Update with new values
    active->setVal(tickActive);
    currentTemperature->setVal(tickCurrentTemperature);
    currentHeaterCoolerState->setVal(tickCurrentHeaterCoolerState);
    targetHeaterCoolerState->setVal(tickTargetHeaterCoolerState);
    coolingThresholdTemperature->setVal(tickCoolingThresholdTemperature);
    heatingThresholdTemperature->setVal(tickHeatingThresholdTemperature);
    swingMode->setVal(tickSwingMode);

    LOG1("[Service:AirConditionerRemote] Values updated:\n");
    LOG1("  - Active = %d\n", tickActive);
    LOG1("  - Current Temperature = %d°C\n", tickCurrentTemperature);
    LOG1("  - Current Heater Cooler State = %d\n", tickCurrentHeaterCoolerState);
    LOG1("  - Target Heater Cooler State = %d\n", tickTargetHeaterCoolerState);
    LOG1("  - Cooling Threshold Temperature = %d°C\n", tickCoolingThresholdTemperature);
    LOG1("  - Heating Threshold Temperature = %d°C\n", tickHeatingThresholdTemperature);
    LOG1("  - Swing Mode = %d\n", tickSwingMode);
  }
};
