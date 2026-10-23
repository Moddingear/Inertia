#include <led.hpp>
#include <Arduino.h>
#include <rmtrgb.hpp>

rmtrgb::ws2812_format leds_data[2];
rmtrgb controller;
static const int num_leds = sizeof(leds_data)/sizeof(leds_data[0]);

int changed_index = -1;

void leds::set(int i, uint8_t r, uint8_t g, uint8_t b)
{
    if (i >= num_leds)
    {
        return;
    }
    
    rmtrgb::ws2812_format new_data;
    new_data.r = r;
    new_data.g = g;
    new_data.b = b;

    if (memcmp(&new_data, &leds_data[i], sizeof(new_data)) == 0)
    {
        return;
    }
    
    changed_index = max(i, changed_index);
    leds_data[i] = new_data;
}


void leds::setup()
{
    controller.init(GPIO_NUM_7);
    memset(leds_data, 0, sizeof(leds_data));
    controller.send(leds_data, sizeof(leds_data), false);
}

void leds::loop()
{
    if (!controller.is_transmitting() && changed_index >= 0)
	{
		controller.send(leds_data, (changed_index+1)*sizeof(leds_data[0]), false);
        changed_index = 0;
	}
}
