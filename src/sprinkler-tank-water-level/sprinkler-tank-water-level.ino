// Sprinkler Tank (Water Level)
//
// Water level reporting for sprinkler tank
// Copyright: 2022, Valerian Saliou <valerian@valeriansaliou.name>
// License: Mozilla Public License v2.0 (MPL v2.0)

#include "HomeSpan.h"
#include "sensors.h"

void setup() {
  // 115,200 bauds (for serial console)
  Serial.begin(115200);

  // Force CPU to a lower power frequency
  setCpuFrequencyMhz(80);

  // Setup HomeSpan accessory
  homeSpan.begin(Category::Sprinklers, "Sprinkler Tank", "vsa-industries", "VSA-WT");
  
  homeSpan.setLogLevel(1);

  // QR Code ID and Pairing codes are used for the HomeKit QR Code
  homeSpan.setQRID("VSWT");
  homeSpan.setPairingCode("78915125");

  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::FirmwareRevision("1.0.0");
      new Characteristic::HardwareRevision("1.0.0");
      new Characteristic::Manufacturer("VSA Industries");
      new Characteristic::Model("VSA-WT-A-R1");
      new Characteristic::Name("Sprinkler Tank Water Level");
      new Characteristic::SerialNumber("WT-2022-07-000001");
      
    new Service::IrrigationSystem();
      new Characteristic::Active(1);
      new Characteristic::ProgramMode();
      new Characteristic::InUse();
    
    new WaterTankLevelSensor();
}

void loop() {
  homeSpan.poll();
}
