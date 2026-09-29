#pragma once

#include <gd_esp.pb.h>
#include <functional>

namespace wifi_if 
{
	// UDP packet parsing to protobuf
	bool parsePacket();
	
	void setup();
	
	void loop();

	typedef std::function<bool(const gd_esp_packet)> OnReceiveCallback;

	void register_callback(pb_size_t type, OnReceiveCallback function);
};