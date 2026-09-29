#include <wifi_protobuf.hpp>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <ESPmDNS.h>


WiFiUDP udp;
int udpPort = 12345;

void wifi_if::setup() {
	// Setup web server routes
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	printf("Connecting to WiFi");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		printf(".");
	}
	printf("\nWiFi connected! IP address: %s\n", WiFi.localIP().toString().c_str());
	if(MDNS.begin("rookie"))
	{
		printf("mDNS responder OK\n");
	}
	else
	{
		printf("mDNS responder failed!\n");
	}
	
	udp.begin(udpPort);
	
	printf("UDP server started on port %d\n", udpPort);
}

void wifi_if::loop() {
	parsePacket();
}

bool wifi_if::parsePacket() {
	size_t length = udp.parsePacket();
	if (!length)
	{
		return false;
	}
	printf("Packet available %d!\n", length);
	uint8_t *buffer = reinterpret_cast<uint8_t*>(alloca(length));
	int len = udp.read(buffer, length);
	if (len == 0)
	{
		return false;
	}
	pb_istream_t stream = pb_istream_from_buffer(buffer, len);
	gd_esp_packet decoded;
	if (!pb_decode(&stream, gd_esp_packet_fields, &decoded))
	{
		return false;
	}
	if (decoded.which_data == gd_esp_packet_PID_tag)
	{
		PIDData &pid = decoded.data.PID; //do stuff
		printf("Received PID packet: %d", pid.loop_id);
		if (pid.which_source == PIDData_motor_index_tag)
		{
			printf("@%d", pid.source.motor_index);
		}
		if (pid.which_source == PIDData_name_tag)
		{
			printf("@%s", pid.source.name);
		}
		if (pid.has_settings)
		{
			printf("P%f I%f D%f, ^%f /%f #%f\n", pid.settings.P, pid.settings.I, pid.settings.D, pid.settings.limit, pid.settings.ramp, pid.settings.low_pass);
		}
		else
		{
			printf("\n");
		}
	}
	if (decoded.which_data == gd_esp_packet_pos_tag)
	{
		printf("Received position packet: X%f Y%f R%f\n", decoded.data.pos.X, decoded.data.pos.Y, decoded.data.pos.R);
	}
	return true;
}

void wifi_if::register_callback(pb_size_t type, OnReceiveCallback function)
{

}