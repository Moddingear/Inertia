#pragma once

#include <gd_esp.pb.h>
#include <functional>
#define USE_ENET PB_BUFFER_ONLY
#if USE_ENET
//#include <enet/enet.h>
#else
#include <IPAddress.h>
#endif

namespace wifi_if 
{

	#if USE_ENET
	typedef uint32_t IPAddress;
	typedef void *client_number_type;
	#else
	typedef uint8_t client_number_type;
	#endif
	struct wifi_client
	{
		IPAddress address;
		uint16_t port;
	};

	// UDP packet parsing to protobuf
	bool parsePacket();
	
	void setup();
	
	void loop();


	typedef std::function<bool(const gd_esp_packet&, client_number_type)> OnReceiveCallback;

	void register_callback(pb_size_t type, OnReceiveCallback function);

	void broadcast(const gd_esp_packet &packet);

	void send(const gd_esp_packet &packet, client_number_type client_number);
};