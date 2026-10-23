#include <motor_enumerate.hpp>
#include <can_interface.hpp>
#include <wifi_protobuf.hpp>
#include <map>
#include <Arduino.h>
#include <led.hpp>

static std::map<uint8_t, motor_enumeration::motor> motors;
static uint64_t last_new_motor_time = 0;
static bool foc_inited = false;
static int foc_started = -1; //-1 when not startedm index of held button when started
static uint64_t last_hello = 0;

struct PIDRequest
{
	wifi_if::client_number_type source_client;
	gd_esp_packet message;
	uint64_t request_time;
};
std::map<uint8_t, PIDRequest> PIDRequests;

wifi_if::client_number_type identification_client;

void motor_enumeration::setup()
{
	last_new_motor_time = millis();
	can_if::register_callback(RobotCAN::PacketType::HELLO, &motor_enumeration::OnHelloCallback, 0);
	can_if::register_callback(RobotCAN::PacketType::MOTOR_FEEDBACK, &motor_enumeration::OnMotorFeedback, sizeof(RobotCAN::Motor_feedback)); 
	can_if::register_callback(RobotCAN::PacketType::PID_SETTING, &motor_enumeration::OnCANPIDData, sizeof(RobotCAN::PIDTunePacket)); //only accept full packet with data
	can_if::register_callback(RobotCAN::PacketType::IDENTIFY, &motor_enumeration::OnMotorIdentification, sizeof(RobotCAN::stm32_identification_fdpacket)); //only accept full packet with data

	wifi_if::register_callback(gd_esp_packet_bootloader_jump_tag, &motor_enumeration::OnBootloaderJump);
	wifi_if::register_callback(gd_esp_packet_PID_tag, &motor_enumeration::OnWiFiPIDData);
	wifi_if::register_callback(gd_esp_packet_network_map_tag, &motor_enumeration::OnNetworkMap);
	wifi_if::register_callback(gd_esp_packet_motor_identification_tag, &motor_enumeration::OnIdentify);
}

uint8_t feedbackidx = 0;

void update_motor_led()
{
	leds::set(0, std::min<uint8_t>(motors.size()*64, 255), foc_inited ? 255:0, feedbackidx);
}

void motor_enumeration::loop()
{
	if (last_new_motor_time + 6000 < millis() && !foc_inited && motors.size() > 0)
	{
		printf("Starting FOC, %d motors\n", motors.size());
		for (auto &&i : motors)
		{
			//auto message = RobotCAN::PacketType::INIT_FOC;
			printf("Starting motor %d\n", i.first);
			can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::INIT_FOC, i.first, HUB_INDEX, false), nullptr, 0);
		}
		foc_inited = true;
		update_motor_led();
	}
	#if 0
	if (last_hello + 6000 > millis())
	{
		last_hello = millis();
		RobotCAN::hello_packet message;
		can_if::send(RobotCAN::discriminator_to_id(0, RobotCAN::IDOffset::to_motor), &message, sizeof(message));
	}
	#endif
}

motor_enumeration::motor& get_motor(uint16_t discriminator)
{
	return motors[discriminator];
}


bool motor_enumeration::OnHelloCallback(const ComplexCanRxHeader& packet, const uint8_t* data)
{
	RobotCAN::ID_extended header = packet.Identifier;
	uint16_t discriminator = header.motor_index;
	auto entry = motors.find(discriminator);
	if (entry == motors.end())
	{
		motor new_motor;
		new_motor.last_transmission = millis();
		motors[discriminator] = new_motor;
		last_new_motor_time = millis();
		printf("Enumeration : Got a motor with discriminator %d\n", discriminator);
	}
	else
	{
		entry->second.last_transmission = millis();
	}
	update_motor_led();
	return true;
}

bool motor_enumeration::OnMotorFeedback(const ComplexCanRxHeader& packet, const uint8_t* data)
{
	auto message = reinterpret_cast<const RobotCAN::Motor_feedback*>(data);
	RobotCAN::ID_extended header = packet.Identifier;
	uint16_t discriminator = header.motor_index;
	//printf("Feedback from motor %d, angle %frad, temperature %fdegC, current %dcA\n", discriminator, message->angle, message->temperature/2.f, message->current);
	auto &motor = motors[discriminator];
	motor.temperature = message->temperature/2.f;
	motor.angle = message->angle;
	feedbackidx++;
	update_motor_led();
	return true;
}

bool motor_enumeration::OnCANPIDData(const ComplexCanRxHeader& packet, const uint8_t* data)
{
	auto message = reinterpret_cast<const RobotCAN::PIDTunePacket*>(data);
	RobotCAN::ID_extended header = packet.Identifier;
	uint16_t discriminator = header.motor_index;
	auto entry = PIDRequests.find(discriminator);
	if (entry == PIDRequests.end())
	{
		return true;
	}
	auto &pid = entry->second.message.data.PID;
	(&pid.settings.P)[static_cast<uint8_t>(message->parameter)] = message->new_value;
	for (size_t i = 0; i < sizeof(pid.settings) / sizeof(pid.settings.P); i++)
	{
		if (isnanf((&pid.settings.P)[i]))
		{
			return true;
		}
	}
	printf("PID request completed for %d\n", entry->first);
	wifi_if::send(entry->second.message, entry->second.source_client);

	PIDRequests.erase(entry);
	return true;
}

bool motor_enumeration::OnMotorIdentification(const ComplexCanRxHeader& packet, const uint8_t* data)
{
	auto message = reinterpret_cast<const RobotCAN::stm32_identification_fdpacket*>(data);
	RobotCAN::ID_extended header = packet.Identifier;
	gd_esp_packet wmessage;
	wmessage.which_data = gd_esp_packet_motor_identification_tag;
	wmessage.data.motor_identification.discriminator = header.motor_index;
	if (message->family == RobotCAN::chip_family::STM32)
	{
		//sprintf doesn't know string length ;(
		char* writeptr = wmessage.data.motor_identification.name;
		strcpy(writeptr, "STM32");
		writeptr += 5;
		memcpy(writeptr, message->subfamily, sizeof(message->subfamily));
		writeptr += sizeof(message->subfamily);
		writeptr += sprintf(writeptr, " X%d Y%d W%d B", 
			message->uid.x, 
			message->uid.y, 
			message->uid.wafer_num);
		memcpy(writeptr, message->uid.lot_number, sizeof(message->uid.lot_number));
		writeptr += sizeof(message->uid.lot_number);
		*writeptr = '\0';
		//printf("UID as printed is %s\n", wmessage.data.motor_identification.name);
	}
	wifi_if::send(wmessage, identification_client);
	return true;
}

bool motor_enumeration::OnBootloaderJump(const gd_esp_packet &packet, wifi_if::client_number_type number)
{
	//auto message = RobotCAN::PacketType::ENTER_BOOTLOADER;
	can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::ENTER_BOOTLOADER, packet.data.bootloader_jump.discriminator, HUB_INDEX, false), nullptr, 0);
	return true;
}

bool motor_enumeration::OnWiFiPIDData(const gd_esp_packet &packet, wifi_if::client_number_type number)
{
	const PIDData &pid = packet.data.PID; //do stuff
	printf("Received PID packet: %d", pid.loop_id);

	const static std::map<PIDData_Loop, RobotCAN::PIDTunePacket::loop_select> motor_loops = {
		{PIDData_Loop_Current_D, 	RobotCAN::PIDTunePacket::loop_select::current_d},
		{PIDData_Loop_Current_Q, 	RobotCAN::PIDTunePacket::loop_select::current_q},
		{PIDData_Loop_Velocity, 	RobotCAN::PIDTunePacket::loop_select::velocity},
		{PIDData_Loop_Angle, 		RobotCAN::PIDTunePacket::loop_select::angle},
	};
	auto entry = motor_loops.find(pid.loop_id);
	bool is_can_target = entry != motor_loops.end() && pid.which_source == PIDData_motor_index_tag && pid.source.motor_index <= UINT8_MAX;

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
	if (is_can_target && !pid.has_settings)
	{
		PIDRequest request;
		request.source_client = number;
		request.request_time = millis();
		request.message.which_data = gd_esp_packet_PID_tag;
		auto &spid = request.message.data.PID;
		spid = pid;
		spid.has_settings = true;
		for (size_t i = 0; i < sizeof(pid.settings) / sizeof(pid.settings.P); i++)
		{
			(&spid.settings.P)[i] = NAN;
		}
		PIDRequests[pid.source.motor_index] = request;
	}
	if (is_can_target)
	{
		RobotCAN::PIDTunePacket message;
		message.loop = entry->second;
		for (size_t i = 0; i < sizeof(pid.settings) / sizeof(pid.settings.P); i++)
		{
			if (pid.has_settings)
			{
				message.new_value = (&pid.settings.P)[i];
				if (isnanf(message.new_value))
				{
					continue;
				}
				message.parameter = static_cast<RobotCAN::PIDTunePacket::parameter_select>(i);
				can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::PID_SETTING, packet.data.bootloader_jump.discriminator, HUB_INDEX, false), &message, sizeof(message));
			}
			else
			{
				message.parameter = static_cast<RobotCAN::PIDTunePacket::parameter_select>(i);
				can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::PID_SETTING, packet.data.bootloader_jump.discriminator, HUB_INDEX, false), &message, sizeof(message) - sizeof(RobotCAN::PIDTunePacket::new_value));
			}
		}
	}
	return true;
}

bool motor_enumeration::OnNetworkMap(const gd_esp_packet &packet, wifi_if::client_number_type number)
{
	gd_esp_packet response = packet;
	auto &nm = response.data.network_map;
	for (auto &&i : motors)
	{
		nm.discriminators.bytes[nm.discriminators.size++] = i.first;
	}
	printf("Network map requested, sending back %d bytes\n", nm.discriminators.size);
	wifi_if::send(response, number);
	return true;
}

bool motor_enumeration::OnIdentify(const gd_esp_packet &packet, wifi_if::client_number_type number)
{
	identification_client = number;
	printf("Got identification request from %d for %d\n", number, packet.data.motor_identification.discriminator);
	can_if::send(RobotCAN::ID_extended(0, RobotCAN::PacketType::IDENTIFY, packet.data.motor_identification.discriminator, HUB_INDEX, false), nullptr, 0);
	return true;
}