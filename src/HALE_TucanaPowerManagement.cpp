#include "HALE_TucanaPowerManagement.h"

TucanaPowerManagement::TucanaPowerManagement() {}

bool TucanaPowerManagement::begin(TwoWire* i2cBus, bool addrA0, bool addrA1,
                                  bool addrA2) {
    this->i2cBus = i2cBus;

    i2cBus->begin();
    i2cBus->setClock(10000);

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

#if TUCANA_POWER_MANAGEMENT_DEBUG
    Serial.println("Tucana Battery Management - initialization complete");
#endif

    // Setup analog digital converters
    MCP342x::generalCallReset();
    lowPowerADC.configure(ch2Config);
    delay(1000);  // MC342x needs 300us to settle

    // i2cBus->requestFrom(lowPowerADCAddr, (uint8_t)3);

    // TODO: proper debugging support
    // if (i2cBus->available()) {
    //     Serial.print("No ADC found at address ");
    //     Serial.println(lowPowerADCAddr, HEX);
    // } else {
    //     Serial.println("Initialized low-power ADC");
    // }

    // First time loop() is called start a conversion
    startConversion = true;

    return true;
}

bool TucanaPowerManagement::read_low_stat() {
    return i2cDevice.digitalRead(TUCANA_POWER_MANAGEMENT_LOW_STAT_PIN);
}

bool TucanaPowerManagement::read_high_stat() {
    return i2cDevice.digitalRead(TUCANA_POWER_MANAGEMENT_HIGH_STAT_PIN);
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
    if (useQuickDisconnect != prevHighPowerControl) {
        i2cDevice.digitalWrite(TUCANA_POWER_MANAGEMENT_HIGH_CTL_PIN,
                               useQuickDisconnect ? HIGH : LOW);
        prevHighPowerControl = useQuickDisconnect;

#if TUCANA_POWER_MANAGEMENT_DEBUG
        SerialDebugger::debugPrintln(
            "Tucana Power Management - set low control pin to " +
            String(useQuickDisconnect));
#endif
    }
}

double low_power_adc_conversion(int32_t rawReading, int pgaGain,
                                int bitResolution) {
    // 1. Calculate the LSB (Least Significant Bit) voltage based on resolution
    // MCP3427 reference is always 2.048V
    double vRef = 2.048;
    double maxCounts =
        std::pow(2, bitResolution - 1);  // Signed 16-bit = 32768 counts

    // 2. Convert raw reading to voltage at the ADC pin
    double vAtPin = (rawReading * vRef) / (maxCounts * pgaGain);

    // 3. Reverse the Voltage Divider
    // R1 = 10,000 ohms, R2 = 39,000 ohms (the one being measured across)
    const double R1 = 10000.0;
    const double R2 = 39000.0;

    double dividerRatio = (R1 + R2) / R2;  // (10k + 39k) / 39k = ~1.256
    double vOriginal = vAtPin * dividerRatio;

    return vOriginal;
}

int TucanaPowerManagement::read_adc() {
    int32_t value = 0;
    uint8_t err;

    if (startConversion) {
        Serial.println("Convert");
        err = lowPowerADC.convert(ch2Config);
        if (err) {
            Serial.print("Convert error: ");
            Serial.println(err);
        }
        startConversion = false;
    }

    err = lowPowerADC.read(value, status);
    if (!err && status.isReady()) {
        // For debugging purposes print the return value.
        Serial.print("Value: ");
        Serial.println(value);
        Serial.print("Config: 0x");
        Serial.println((int)ch2Config, HEX);
        Serial.print("Convert error: ");
        Serial.println(err);
        startConversion = true;
        Serial.print("Original voltage: ");
        Serial.println(low_power_adc_conversion(value, 1, 12));
    }

    return 0;
}
