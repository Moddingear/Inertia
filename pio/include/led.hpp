#pragma once

#include <stdint.h>

namespace leds{
    void set(int i, uint8_t r, uint8_t g, uint8_t b);

    void setup();
    void loop();
}