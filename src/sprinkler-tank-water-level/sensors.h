// Sprinkler Tank (Water Level)
//
// Water level reporting for sprinkler tank
// Copyright: 2022, Valerian Saliou <valerian@valeriansaliou.name>
// License: Mozilla Public License v2.0 (MPL v2.0)

const int POLL_EVERY_MILLISECONDS = 600000; // 10 minutes

const float WATER_TANK_SENSOR_OFFSET_DISTANCE = 1.0; // 1.0 centimeters
const float WATER_TANK_FILL_EMPTY_DISTANCE = 28.0; // 28.0 centimeters

const unsigned int WATER_LEVEL_PROBE_DELAY = 10; // 1/100 second
const unsigned int WATER_LEVEL_PROBE_SAMPLES = 10;

const int WATER_LEVEL_SENSOR_PIN_TRIGGER = 22; // Yellow cable
const int WATER_LEVEL_SENSOR_PIN_ECHO = 21; // Blue cable

struct WaterTankLevelSensor : Service::BatteryService {
  bool valuesInitialized;
  SpanCharacteristic *waterLevel;
  SpanCharacteristic *statusLowBattery;

  WaterTankLevelSensor() : Service::BatteryService() {
    // Mark values as not initialized
    valuesInitialized = false;
    
    // Configure water level characteristics
    new Characteristic::ChargingState(0);

    waterLevel = new Characteristic::BatteryLevel(100);
    waterLevel->setRange(0, 100);

    statusLowBattery = new Characteristic::StatusLowBattery(0);

    // Configure water level sensor pins
    pinMode(WATER_LEVEL_SENSOR_PIN_TRIGGER, OUTPUT);
    pinMode(WATER_LEVEL_SENSOR_PIN_ECHO, INPUT);
  }

  void loop() {
    // Wait until next poll is possible
    // Warning: never block this main loop with a delay(), as this will cause \
    //   the accessory from being marked as 'not responding' on the Home app.
    if (valuesInitialized == false || waterLevel->timeVal() > POLL_EVERY_MILLISECONDS){
      LOG1("[Sensor:WaterTankLevel] Loop tick in progress...\n");
  
      // Check current water level
      pollAndUpdate();
  
      LOG1("[Sensor:WaterTankLevel] Loop tick done, next in %dms\n", POLL_EVERY_MILLISECONDS);

      // Mark values as initialized (used for the first pass only)
      valuesInitialized = true;
    }
  }

  void pollAndUpdate() {
    unsigned int tickWaterLevel = probeWaterLevel();
    bool isLowLevel = tickWaterLevel <= 20 ? true : false;

    waterLevel->setVal(tickWaterLevel);
    statusLowBattery->setVal(isLowLevel ? 1 : 0);

    LOG1("[Sensor:WaterTankLevel] Water level updated:\n");
    LOG1("  - Level = %d%%\n", tickWaterLevel);
    if (isLowLevel) {
      LOG1("  - (!) Low water level\n");
    }
  }

  unsigned int probeWaterLevel() {
    // Acquire multiple measurement samples
    unsigned int nextSampleIndex = 0;
    float samples[WATER_LEVEL_PROBE_SAMPLES] = {};

    while (nextSampleIndex < WATER_LEVEL_PROBE_SAMPLES) {
      // Probe sample
      samples[nextSampleIndex] = probeWaterLevelSample(nextSampleIndex + 1);

      // Increment next sample index
      nextSampleIndex++;

      // Hold on to probe next sample
      // Warning: make sure this delay is kept very short, as not to block \
      //   the main loop.
      delay(WATER_LEVEL_PROBE_DELAY);
    }

    // Sort samples (required for the median value)
    floatQuickSort(samples, 0, WATER_LEVEL_PROBE_SAMPLES - 1);

    // Acquire the median value (this makes sure outliers are not considered)
    float tickWaterLevelMedian = samples[(WATER_LEVEL_PROBE_SAMPLES / 2) - 1];

    // Round up water level to an integer
    unsigned int tickWaterLevel = round(tickWaterLevelMedian);

    return tickWaterLevel;
  }

  float probeWaterLevelSample(unsigned int sampleIndex) {
    // Wake up the sensor (ie. trigger)
    digitalWrite(WATER_LEVEL_SENSOR_PIN_TRIGGER, LOW);
    delayMicroseconds(5);
    digitalWrite(WATER_LEVEL_SENSOR_PIN_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(WATER_LEVEL_SENSOR_PIN_TRIGGER, LOW);

    pinMode(WATER_LEVEL_SENSOR_PIN_ECHO, INPUT);

    // Acquire echo duration
    unsigned long durationSample = pulseIn(WATER_LEVEL_SENSOR_PIN_ECHO, HIGH);

    // Duration is zero? Report fault
    if (durationSample == 0) {
      LOG0("[Sensor:WaterTankLevel] Water level sample failed! Is the sensor connected?\n");
      
      return 0.0;
    }

    // Convert the time to echo into a distance
    float distanceSample = ((float)durationSample / 2) / 29.1;

    // Apply sensor offset from water at 100% level
    distanceSample -= WATER_TANK_SENSOR_OFFSET_DISTANCE;

    // Compute water level percentage
    // Important: restrict between [0.0; 100.0]
    float levelPercentSample = 100.0 - max(0.0, min((distanceSample / WATER_TANK_FILL_EMPTY_DISTANCE) * 100.0, 100.0));

    LOG1("[Sensor:WaterTankLevel] Water level sample #%d captured = %.2f%% (%dµs <-> %.2fcm)\n", sampleIndex, levelPercentSample, durationSample, distanceSample);

    return levelPercentSample;
  }

  void floatQuickSort(float values[], int left, int right) {
    // Initial values
    float tmp;
    float pivot = values[(left + right) / 2];
    int i = left, j = right;

    // Proceed partition
    while (i <= j) {
      while (values[i] < pivot) {
        i++;
      }
      
      while (values[j] > pivot) {
        j--;
      }
      
      if (i <= j) {
        tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
        
        i++;
        j--;
      }
    };

    // Recurse
    if (left < j) {
      floatQuickSort(values, left, j);
    }

    if (i < right) {
      floatQuickSort(values, i, right);
    }
  }
};
