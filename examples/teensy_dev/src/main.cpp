#include <Arduino.h>

#define TUCANA_POWER_MANAGEMENT_DEBUG true
#include <HALE_TucanaPowerManagement.h>

#include <Wire.h>

TucanaPowerManagement tucana;

void setup() {
    Serial.begin(115200);

    // Wait for serial monitor to connect (up to 10 seconds)
    while (!Serial || millis() < 10000) continue;

    tucana.begin(&Wire1, 0x00, 0x00, true, true, true, 12);
}

void loop() {
    Serial.println("Low stat is: " + String(tucana.read_low_stat()));
    Serial.println("High stat is: " + String(tucana.read_low_stat()));

    delay(2000);

    Serial.println("Setting low power source to QD");
    tucana.set_low_power_ctl(true);

    delay(2000);

    Serial.println("Setting high power source to QD");
    tucana.set_high_power_ctl(true);

    delay(2000);

    Serial.println("Low stat is: " + String(tucana.read_low_stat()));
    Serial.println("High stat is: " + String(tucana.read_low_stat()));

    delay(2000);

    Serial.println("Setting low power source to battery");
    tucana.set_low_power_ctl(false);

    delay(2000);

    Serial.println("Setting high power source to battery");
    tucana.set_high_power_ctl(false);

    delay(2000);
}