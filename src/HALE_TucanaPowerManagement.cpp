#include "HALE_TucanaPowerManagement.h"

TucanaPowerManagement::TucanaPowerManagement() {}

bool TucanaPowerManagement::begin(TwoWire* i2cBus, uint8_t lowBatteryReadPin,
                                  uint8_t highBatteryReadPin, bool addrA0,
                                  bool addrA1, bool addrA2,
                                  int analogReadResolution) {
    Arduino_h::analogReadResolution(analogReadResolution);

    this->i2cBus = i2cBus;
    this->lowBatteryReadPin = lowBatteryReadPin;
    this->highBatteryReadPin = highBatteryReadPin;

    // The maximum value returned from an analog read will be 2^resolution
    this->maxAnalogReading = pow(2, analogReadResolution);

    i2cBus->begin();

#if TUCANA_POWER_MANAGEMENT_DEBUG
    Serial.println("Tucana Battery Management - beginning initialization");
#endif

    // Construct I2C address. See Tucana documentation for more info.
    // ADDRESS FORMAT: [ 0 1 0 0 A2 A1 A0 ]
    uint8_t i2cAddress = 0x20 | (addrA0 << 0) | (addrA1 << 1) | (addrA2 << 2);

    if (!i2cDevice.begin_I2C(i2cAddress, i2cBus)) {
        Serial.println(
            "Tucana Battery Management - failed to init Adafruit_MCP23X08 on "
            "address " +
            String(i2cAddress));

        return false;
    }

#if TUCANA_POWER_MANAGEMENT_DEBUG
    SerialDebugger::debugPrintln(
        "Tucana Battery Management - configuring MCP23008 control pins");
#endif

    // Digital I/O pins of the battery control board,
    // controlled over I2C w/ MCP23008
    i2cDevice.pinMode(TUCANA_POWER_MANAGEMENT_LOW_CTL_PIN, OUTPUT);
    i2cDevice.pinMode(TUCANA_POWER_MANAGEMENT_HIGH_CTL_PIN, OUTPUT);
    i2cDevice.pinMode(TUCANA_POWER_MANAGEMENT_LOW_STAT_PIN, INPUT);
    i2cDevice.pinMode(TUCANA_POWER_MANAGEMENT_HIGH_STAT_PIN, INPUT);

#if TUCANA_POWER_MANAGEMENT_DEBUG
    SerialDebugger::debugPrintln(
        "Tucana Battery Management - setting default MCP23008 control pin "
        "outputs");
#endif

    // Default outputs
    i2cDevice.digitalWrite(TUCANA_POWER_MANAGEMENT_LOW_CTL_PIN, LOW);
    i2cDevice.digitalWrite(TUCANA_POWER_MANAGEMENT_HIGH_CTL_PIN, LOW);

#if TUCANA_POWER_MANAGEMENT_DEBUG
    SerialDebugger::debugPrintln(
        "Tucana Battery Management - initializing microcontroller analog "
        "battery reading pins");
#endif

    // Configure analog battery input pins
    pinMode(this->lowBatteryReadPin, INPUT);
    pinMode(this->highBatteryReadPin, INPUT);

#if TUCANA_POWER_MANAGEMENT_DEBUG
    Serial.println("Tucana Battery Management - initialization complete");
#endif

    return true;
}

bool TucanaPowerManagement::read_low_stat() {
    return i2cDevice.digitalRead(TUCANA_POWER_MANAGEMENT_LOW_STAT_PIN);
}

bool TucanaPowerManagement::read_high_stat() {
    return i2cDevice.digitalRead(TUCANA_POWER_MANAGEMENT_HIGH_STAT_PIN);
}

float TucanaPowerManagement::read_low_battery_voltage() {
    int rawPinInput = analogRead(lowBatteryReadPin);

    // Convert to 0-3.3v range (the voltage the MCP23008 reads at)
    float readVoltage = (rawPinInput / (float)maxAnalogReading) * 3.3;

    // Undo the Tucana voltage divider circuit to get the actual battery voltage
    return readVoltage * TUCANA_POWER_MANAGEMENT_LOW_READING_CONVERSION;
}

float TucanaPowerManagement::read_high_battery_voltage() {
    int rawPinInput = analogRead(highBatteryReadPin);

    // Convert to 0-3.3v range (the voltage the MCP23008 reads at)
    float readVoltage = (rawPinInput / (float)maxAnalogReading) * 3.3;

    // Undo the Tucana voltage divider circuit to get the actual battery voltage
    return readVoltage * TUCANA_POWER_MANAGEMENT_HIGH_READING_CONVERSION;
}

void TucanaPowerManagement::set_low_power_ctl(bool useQuickDisconnect) {
    if (useQuickDisconnect != prevLowPowerControl) {
        i2cDevice.digitalWrite(TUCANA_POWER_MANAGEMENT_LOW_CTL_PIN,
                               useQuickDisconnect ? HIGH : LOW);
        prevLowPowerControl = useQuickDisconnect;

#if TUCANA_POWER_MANAGEMENT_DEBUG
        SerialDebugger::debugPrintln(
            "Tucana Power Management - set low control pin to " +
            String(useQuickDisconnect));
#endif
    }
}

void TucanaPowerManagement::set_high_power_ctl(bool useQuickDisconnect) {
    // Same as above
    if (useQuickDisconnect != prevLowPowerControl) {
        i2cDevice.digitalWrite(TUCANA_POWER_MANAGEMENT_LOW_CTL_PIN,
                               useQuickDisconnect ? HIGH : LOW);
        prevLowPowerControl = useQuickDisconnect;

#if TUCANA_POWER_MANAGEMENT_DEBUG
        SerialDebugger::debugPrintln(
            "Tucana Power Management - set low control pin to " +
            String(useQuickDisconnect));
#endif
    }
}
