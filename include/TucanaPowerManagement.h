#pragma once

#include <Arduino.h>

class TucanaPowerManagement {
   public:
    TucanaPowerManagement();
    void begin();
    int add(int a, int b);
};