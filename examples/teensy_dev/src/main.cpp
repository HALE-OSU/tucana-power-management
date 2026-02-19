#include <Arduino.h>

#define TUCANA_POWER_MANAGEMENT_DEBUG true
#include <HALE_TucanaPowerManagement.h>

#include <Wire.h>

TucanaPowerManagement tucana;

#define CYCLE_CTL false
#define READ_ADC true

void setup() {
    Serial.begin(115200);

    // Wait for serial monitor to connect (up to 10 seconds)
    while (!Serial || millis() < 10000) continue;

    byte error, address;
    int nDevices;

    Serial.println("Scanning i2c bus ...");

    Wire2.begin();
    Wire2.setClock(50000);

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // We are using the return value of Wire.endTransmission to determine
        // if
        // a device responded. A return value of 0 means success (device
        // acknowledged).
        Wire2.beginTransmission(address);
        error = Wire2.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            // An unknown error occurred.
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("done\n");
    }

    // // Pause program once scan is complete
    // while (1);

    tucana.begin(&Wire2, true, true, true);
}

void loop() {
#if READ_ADC
    // tucana.read_adc();
    // tucana.update();
    tucana.read_adc();
#endif

#if CYCLE_CTL
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
#endif
}