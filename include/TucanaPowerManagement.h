#pragma once

#include <Arduino.h>
#include <Adafruit_MCP23X08.h>

class TucanaPowerManagement {
   public:
    TucanaPowerManagement();
    void begin();
    int add(int a, int b);
};