#pragma once

#include <inttypes.h>

#define PACKED __attribute__((packed))

namespace RobotCAN
{

static const uint32_t protocol_version = 0;
static const uint32_t master_address_low_priority = 2047, master_address_high_priority = 0;


enum class PacketType : uint8_t
{
	STOP_ALL, //emergency stop
	VERSION_CHECK, //check protocol version
	ERROR_PACKET, //report error

	MOTOR_TARGET, 
	MOTOR_DRIVE_MODE,

	MOTOR_FEEDBACK,

	INIT_FOC,
	ENABLE_FOC,

	PID_SETTING,

	HELLO,
	ALIVE
};

struct PACKED Version_check
{
	PacketType id = PacketType::VERSION_CHECK;
	uint32_t version = protocol_version;
};

struct PACKED Error_packet
{
	enum class errors : uint16_t
	{
		VERSION_MISMATCH,
		DISCRIMINATOR_OVERLAP,

		NOT_INITIATED,

		OVERCURRENT,
		OVERTEMP,
		UNDERVOLTAGE
	};
	PacketType id = PacketType::ERROR_PACKET;
	errors error;
};


struct PACKED Motor_target_packet  //MOTOR_TARGET
{
	PacketType id = PacketType::MOTOR_TARGET;
	float target;
};

struct PACKED Motor_mode_packet  //MOTOR_TARGET
{
	PacketType id = PacketType::MOTOR_DRIVE_MODE;
	enum class mode_select : uint8_t
	{
		torque, velocity, angle
	};
	mode_select mode;
};

struct PACKED PIDTunePacket
{
	enum class parameter_select : uint8_t
	{
		P, I, D, Ramp, Limit, LPF
	};
	enum class loop_select : uint8_t
	{
		current_d, current_q, velocity, angle
	};
	PacketType id = PacketType::PID_SETTING;
	parameter_select parameter;
	loop_select loop;
	float new_value; //optional, if not included then it's a request
};

struct PACKED Motor_feedback
{
	PacketType id = PacketType::MOTOR_FEEDBACK;
	uint8_t temperature; //in half degrees celsius
	int16_t current;
	float angle;
};

//Hello packet : On powering up, or after 5 seconds without any data, we send an hello to the host

struct PACKED hello_packet
{
	PacketType id = PacketType::HELLO;
	uint16_t discriminator;
};


};

#undef PACKED