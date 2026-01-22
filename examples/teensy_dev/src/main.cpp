#include <Arduino.h>
#include <TucanaPowerManagement.h>

TucanaPowerManagement tucana;

void setup() {
    Serial.begin(115200);
    tucana.begin();
}

void loop() {
    Serial.println("Hello world!");
}