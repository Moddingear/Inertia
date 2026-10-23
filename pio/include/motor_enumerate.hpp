#pragma once

#include <esp_twai_types.h>
#include <gd_esp.pb.h>
#include <IPAddress.h>
#include <ComplexCAN.h>
#include <wifi_protobuf.hpp>

namespace motor_enumeration
{
	struct motor
	{
		uint64_t last_transmission;
		float angle;
		float temperature;
	};

	void setup();
	void loop();

	motor& get_motor(uint16_t discriminator);

	bool OnHelloCallback(const ComplexCanRxHeader& packet, const uint8_t* data);
	bool OnMotorFeedback(const ComplexCanRxHeader& packet, const uint8_t* data);
	bool OnCANPIDData(const ComplexCanRxHeader& packet, const uint8_t* data);
	bool OnMotorIdentification(const ComplexCanRxHeader& packet, const uint8_t* data);

	bool OnBootloaderJump(const gd_esp_packet &packet, wifi_if::client_number_type number);
	bool OnWiFiPIDData(const gd_esp_packet &packet, wifi_if::client_number_type number);
	bool OnNetworkMap(const gd_esp_packet &packet, wifi_if::client_number_type number);
	bool OnIdentify(const gd_esp_packet &packet, wifi_if::client_number_type number);
} // namespace motor_enumeration
