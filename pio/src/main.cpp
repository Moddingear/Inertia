#include <Arduino.h>
#include <can_interface.hpp>
#include <wifi_protobuf.hpp>
#include <motor_enumerate.hpp>
#include <lidar.hpp>
#include <led.hpp>



void setup()
{
	delay(2000);
	leds::setup();
	can_if::setup();
	wifi_if::setup();
	motor_enumeration::setup();
	lidar::setup();

}

void loop() 
{
	wifi_if::loop();
	can_if::loop();
	motor_enumeration::loop();
	leds::loop();
	lidar::loop();
	
	yield();
	//vTaskDelay(pdMS_TO_TICKS(1));

}