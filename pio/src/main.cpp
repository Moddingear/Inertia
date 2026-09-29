#include <Arduino.h>
#include <can_interface.hpp>
#include <wifi_protobuf.hpp>

uint64_t last_new_motor_time = 0;
bool foc_inited = false;
int foc_started = -1; //-1 when not startedm index of held button when started



void setup()
{
	can_if::setup();
	wifi_if::setup();
}

void loop() 
{
	wifi_if::loop();
	can_if::loop();
	yield();
	//vTaskDelay(pdMS_TO_TICKS(1));
}