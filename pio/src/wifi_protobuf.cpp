#include <wifi_protobuf.hpp>
#include <WiFi.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <ESPmDNS.h>
#include <map>
#include <vector>
#include <led.hpp>
#include <enet/enet.h>
#ifndef PB_BUFFER_ONLY
#error "Define PB_BUFFER_ONLY"
#endif

#include <lidar.hpp>

ENetHost *server;
ENetEvent event;
static const int udpPort = 12345;

uint64_t last_receive_millis = 0;

static std::map<pb_size_t, wifi_if::OnReceiveCallback> CallbackMap;

void update_wifi_led()
{
	uint64_t time_since_last_rx = millis() - last_receive_millis;
	uint64_t fade = time_since_last_rx > 1000 ? 0 : 1000 - time_since_last_rx;
	uint8_t greenpower = fade * 255 / 1000;
	leds::set(1, WiFi.isConnected() ? 0 : 255, greenpower, 0);
}

void wifi_if::setup() {
	// Setup web server routes
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	#if 0
	printf("Connecting to WiFi");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		printf(".");
	}
	printf("\nWiFi connected! IP address: %s\n", WiFi.localIP().toString().c_str());
	#endif
	if(MDNS.begin("rookie"))
	{
		printf("mDNS responder OK\n");
	}
	else
	{
		printf("mDNS responder failed!\n");
	}
	if (enet_initialize())
	{
		printf("Failed to init ENet!\n");
	}
	ENetAddress address;
	address.host = ENET_HOST_ANY;
	address.port = udpPort;
	server = enet_host_create(&address, 4, 1, 0, 0);
	if (server == nullptr)
	{
		printf("Failed to create ENet host!\n");
	}
	else
	{
		printf("ENet host started on port %d\n", udpPort);
	}
	
}

void wifi_if::loop() {
	auto status = enet_host_service(server, &event, 0);
	if (status > 0)
	{
		auto address = event.peer->address;
		switch (event.type)
		{
		case ENET_EVENT_TYPE_CONNECT:
			{
				printf("%x:%u connected.\n", 
					address.host,
					address.port);
				if (server->connectedPeers == 1)
				{
					printf("Starting lidar\n");
					lidar::start();
				}
				
			}
			break;
		case ENET_EVENT_TYPE_RECEIVE:
			parsePacket();
			break;
		case ENET_EVENT_TYPE_DISCONNECT:
			{
				printf("%x:%u disconnected.\n", 
					address.host,
					address.port);
				if (server->connectedPeers == 0)
				{
					printf("Stopping lidar\n");
					lidar::stop();
				}
			}
			break;
		default:
			break;
		}
	}
	else if (status < 0)
	{
		printf("enet_host_service fail %d\n", status);
	}
	update_wifi_led();
}

bool wifi_if::parsePacket() {
	client_number_type client_number = event.peer;
	pb_istream_t istream = pb_istream_from_buffer(event.packet->data, event.packet->dataLength);
	gd_esp_packet decoded;
	if (!pb_decode(&istream, gd_esp_packet_fields, &decoded))
	{
		printf("Failed to decode to protobuf ! %s\n", istream.errmsg);
		return false;
	}
	auto entry = CallbackMap.find(decoded.which_data);
	if (entry == CallbackMap.end())
	{
		printf("Missing callback for packet type %d on UDP\n", decoded.which_data);
		return false;
	}
	last_receive_millis = millis();
	enet_packet_destroy(event.packet);
	return entry->second(decoded, client_number);
}

void wifi_if::register_callback(pb_size_t type, OnReceiveCallback function)
{
	CallbackMap[type] = function;
}

ENetPacket* encode_packet(const gd_esp_packet &packet, uint32_t flags)
{
	uint8_t buffer[1024];
	pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if(!pb_encode(&ostream, gd_esp_packet_fields, &packet))
	{
		printf("Failed to encode from protobuf ! %s\n", ostream.errmsg);
		return nullptr;
	}
	ENetPacket *epacket = enet_packet_create(buffer, ostream.bytes_written, flags);
	if (epacket == nullptr)
	{
		printf("Failed to allocate ENet packet!\n");
		return nullptr;
	}
	return epacket;
}

void wifi_if::broadcast(const gd_esp_packet &packet)
{
	auto epacket = encode_packet(packet, ENET_PACKET_FLAG_UNSEQUENCED);
	if (epacket == nullptr)
	{
		return;
	}
	enet_host_broadcast(server, 0, epacket);
}

void wifi_if::send(const gd_esp_packet &packet, client_number_type client_number)
{
	auto epacket = encode_packet(packet, ENET_PACKET_FLAG_RELIABLE);
	if (epacket == nullptr)
	{
		return;
	}
	if (auto error = enet_peer_send(reinterpret_cast<ENetPeer*>(client_number), 0, epacket))
	{
		printf("Failed to send over ENet: %d\n", error);
		enet_packet_destroy(epacket);
	}
}