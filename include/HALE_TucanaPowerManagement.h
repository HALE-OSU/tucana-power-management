#pragma once

#include <Arduino.h>
#include <Adafruit_MCP23X08.h>
#include <Wire.h>
#include <MCP342x.h>

#define TUCANA_POWER_MANAGEMENT_LOW_CTL_PIN 6u
#define TUCANA_POWER_MANAGEMENT_HIGH_CTL_PIN 0u
#define TUCANA_POWER_MANAGEMENT_LOW_STAT_PIN 7u
#define TUCANA_POWER_MANAGEMENT_HIGH_STAT_PIN 1u
#define TUCANA_POWER_MANAGEMENT_LOW_QD_PIN 5u
#define TUCANA_POWER_MANAGEMENT_HIGH_QD_PIN 2u

// From Tucana documentation
// TODO: actual conversions
#define TUCANA_POWER_MANAGEMENT_LOW_READING_CONVERSION 1.0
#define TUCANA_POWER_MANAGEMENT_HIGH_READING_CONVERSION 1.0

class TucanaPowerManagement {
   private:
    TwoWire* i2cBus;
    Adafruit_MCP23X08 i2cDevice;

    const int lowPowerADCAddr = 0x6E;

    MCP342x lowPowerADC = MCP342x(lowPowerADCAddr);
    MCP342x highPowerADC = MCP342x(0x68);

    MCP342x::Config ch1Config =
        MCP342x::Config(MCP342x::channel1, MCP342x::oneShot,
                        MCP342x::resolution16, MCP342x::gain1);

    MCP342x::Config ch2Config =
        MCP342x::Config(MCP342x::channel2, MCP342x::oneShot,
                        MCP342x::resolution16, MCP342x::gain1);

    // TODO: should be per-channel
    MCP342x::Config status;

    bool startedLowAdcConversion = false;
    bool startedHighAdcConversion = false;

    bool prevLowControl = false;
    bool prevHighControl = false;

    float recent_low_batt_voltage = 0;
    float recent_low_qd_voltage = 0;
    float recent_high_batt_voltage = 0;
    float recent_high_qd_voltage = 0;

    void update_voltage_readings();

    float voltage_conversion(int32_t rawReading, float rRead, float rOther);

    float low_voltage_conversion(int32_t rawReading);

    float high_voltage_conversion(int32_t rawReading);

    bool readingLowBatt;
    bool readingHighBatt;

   public:
    /**
     * Add "#define TUCANA_POWER_MANAGEMENT_DEBUG true" before importing this
     * library to enable debug prints
     */
    TucanaPowerManagement();

    /**
     * Completes all required initialization for Tucana. Sets the power source
     * to batteries by default.
     *
     * @param i2cBus - the teensy I2C Tucana is connect to. Either &Wire1 or
     * &Wire2
     * @param addrA0 - true if the A0 (+) address jumper is soldered, false if
     * the A0 (-) address jumper is soldered
     * @param addrA1 - true if the A1 (+) address jumper is soldered, false if
     * the A1 (-) address jumper is soldered
     * @param addrA2 - true if the A2 (+) address jumper is soldered, false if
     * the A2 (-) address jumper is soldered
     *
     * @returns true if initialization is successful, false if not
     */
    bool begin(TwoWire* i2cBus, bool addrA0, bool addrA1, bool addrA2);

    /**
     * Reads the low battery stat pin
     *
     * @returns true if the low power battery is in use or false if the low
     * power quick disconnect power source is in use
     */
    bool read_low_stat();

    /**
     * Reads the high battery stat pin
     *
     * @returns true if the high power battery is in use or false if the high
     * power quick disconnect power source is in use
     */
    bool read_high_stat();

    /**
     * @returns true if the low power QD is connected, false if not
     */
    bool read_low_qd();

    /**
     * @returns true if the high power QD is connected, false if not
     */
    bool read_high_qd();

    /**
     * Sets the state of the low power control pin. Note that this will only
     * update the pin state when a new value is provided, meaning calling this
     * in a loop is not an issue.
     *
     * @param useQuickDisconnect - supply true to set Tucana to use the low
     * power quick disconnect, or false to use the low power battery
     */
    void set_low_power_ctl(bool useQuickDisconnect);

    /**
     * Sets the state of the high power control pin. Note that this will only
     * update the pin state when a new value is provided, meaning calling this
     * in a loop is not an issue.
     *
     * @param useQuickDisconnect - supply true to set Tucana to use the high
     * power quick disconnect, or false to use the high power battery
     */
    void set_high_power_ctl(bool useQuickDisconnect);

    float get_low_batt_voltage();
    float get_low_qd_voltage();

    float get_high_batt_voltage();
    float get_high_qd_voltage();
};