#include <Arduino.h>

#define TUCANA_POWER_MANAGEMENT_DEBUG true
#include <HALE_TucanaPowerManagement.h>

#include <Wire.h>

TucanaPowerManagement tucana;

#define CYCLE_CTL true
// #define READ_ADC true

void setup() {
    Serial.begin(115200);

    // Wait for serial monitor to connect (up to 10 seconds)
    while (!Serial || millis() < 4000) continue;

    byte error, address;
    int nDevices;

    Serial.println("Scanning i2c bus ...");

    Wire.begin();
    Wire.setClock(50000);

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // We are using the return value of Wire.endTransmission to determine
        // if
        // a device responded. A return value of 0 means success (device
        // acknowledged).
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

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

    tucana.begin(&Wire, true, true, true);
}

void loop() {
#if READ_ADC
    // tucana.read_adc();
    // tucana.update();
    Serial.println(tucana.get_low_batt_voltage());
#endif

#if CYCLE_CTL
    Serial.println("Low QD selected: " + String(tucana.low_qd_selected()));
    Serial.println("High QD selected: " + String(tucana.high_qd_selected()));
    Serial.println("Low QD present: " + String(tucana.low_qd_present()));
    Serial.println("High QD present: " + String(tucana.high_qd_present()));

    delay(2000);

    Serial.println("Setting power sources to QD");
    tucana.set_low_power_ctl(true);
    tucana.set_high_power_ctl(true);

    // delay(2000);

    // Serial.println("Setting high power source to QD");
    // tucana.set_high_power_ctl(true);

    delay(2000);

    Serial.println("Low QD selected: " + String(tucana.low_qd_selected()));
    Serial.println("High QD select: " + String(tucana.high_qd_selected()));
    Serial.println("Low QD present: " + String(tucana.low_qd_present()));
    Serial.println("High QD present: " + String(tucana.high_qd_present()));

    delay(2000);

    Serial.println("Setting power sources to battery");
    tucana.set_low_power_ctl(false);
    tucana.set_high_power_ctl(false);

    // delay(2000);

    // Serial.println("Setting high power source to battery");
    // tucana.set_high_power_ctl(false);

    delay(2000);
#endif
}