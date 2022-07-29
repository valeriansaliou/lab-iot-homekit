// Air Conditioner (Remote)
//
// Air conditioner remote controller
// Copyright: 2022, Valerian Saliou <valerian@valeriansaliou.name>
// License: Mozilla Public License v2.0 (MPL v2.0)

#include "HomeSpan.h"
#include "services.h"

void setup() {
  // 115,200 bauds (for serial console)
  Serial.begin(115200);

  // Force CPU to a lower power frequency
  setCpuFrequencyMhz(80);

  // Setup HomeSpan accessory
  homeSpan.begin(Category::AirConditioners, "Air Conditioner", "vsa-industries", "VSA-AC");
  
  homeSpan.setLogLevel(1);

  // QR Code ID and Pairing codes are used for the HomeKit QR Code
  homeSpan.setQRID("VSAC");
  homeSpan.setPairingCode("89104319");

  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::FirmwareRevision("1.0.0");
      new Characteristic::HardwareRevision("1.0.0");
      new Characteristic::Manufacturer("VSA Industries + Crisp X");
      new Characteristic::Model("VSA-AC-A-R1");
      new Characteristic::Name("Air Conditioner Remote");
      new Characteristic::SerialNumber("AC-2022-07-000001");

    new AirConditionerRemote();
}

void loop() {
  homeSpan.poll();
}
