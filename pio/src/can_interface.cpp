
#include <Arduino.h>
#include <driver/twai.h>
#include <CANRobot.hpp>
#include <can_interface.hpp>
#include <map>

#define CAN_TX 36
#define CAN_RX 35
#define CAN_SD 37

struct motor
{
	uint16_t discriminator;
	uint64_t last_transmission;
};


std::vector<motor> motors;

std::map<RobotCAN::PacketType, std::pair<can_if::OnReceiveCallback, uint8_t>> CallbackMap;

void OnReceive(const twai_message_t packet)
{
	if (packet.data_length_code < sizeof(RobotCAN::PacketType))
	{
		return;
	}
	auto type = *reinterpret_cast<const RobotCAN::PacketType*>(&packet.data[0]);
    auto entry = CallbackMap.find(type);
    if (entry == CallbackMap.end())
    {
        printf("Missing callback for packet type %d on CAN\n", type);

    }
    if (packet.data_length_code < entry->second.second)
    {
        printf("Packet too small for %d on CAN (%d)\n", type, packet.data_length_code);
    }
    entry->second.first(packet);
    #if 0
	switch (type)
	{
	case RobotCAN::PacketType::HELLO :
		if (packet.data_length_code == sizeof(RobotCAN::hello_packet))
		{
			auto message = reinterpret_cast<const RobotCAN::hello_packet*>(packet.data);
			uint16_t discriminator = message->discriminator;
			size_t i;
			for (i = 0; i < motors.size(); i++)
			{
				if (motors[i].discriminator == discriminator)
				{
					break;
				}
			}
			if (i == motors.size())
			{
				motor new_motor;
				new_motor.discriminator = discriminator;
				new_motor.last_transmission = millis();
				motors.push_back(new_motor);
				last_new_motor_time = millis();
				printf("Enumeration : Got a motor with discriminator %d\n", discriminator);
			}
			else
			{
				motors[i].last_transmission = millis();
			}
			
		}
		break;
	case RobotCAN::PacketType::MOTOR_FEEDBACK :
		if (packet.data_length_code == sizeof(RobotCAN::Motor_feedback))
		{
			auto message = reinterpret_cast<const RobotCAN::Motor_feedback*>(packet.data);
			//printf("Feedback from motor, angle %frad, temperature %fdegC, current %dcA\n", message->angle, message->temperature/2.f, message->current);
		}
		break;
	default:
		printf("Got an unknown packet, identifier %#d\n", packet.identifier);
		break;
	}
    #endif
}

void can_send(uint32_t id, void* data, uint8_t length, bool single_shot = false)
{
	twai_message_t message;
	memset(&message, 0, sizeof(message));
	message.identifier = id;
	memcpy(message.data, data, length);
	message.data_length_code = length;
	message.ss = single_shot;
	auto err = twai_transmit(&message, portMAX_DELAY);
	if (err != ESP_OK)
	{
		printf("CAN Send fail : %s", esp_err_to_name(err));
	}
	
}

void can_if::setup()
{
    pinMode(CAN_SD, OUTPUT);
	pinMode(CAN_TX, OUTPUT);
	pinMode(CAN_RX, INPUT);
	digitalWrite(CAN_SD, LOW);
	//gpio_dump_io_configuration(stdout, (1ULL<<2) | (3ULL<<43));
	//Serial.begin(115200);
	// Initialize configuration structures using macro initializers
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
	//g_config.bus_off_io = GPIO_NUM_2;
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
	twai_filter_config_t f_config;
	f_config.acceptance_code = UINT16_MAX << 16;
	f_config.acceptance_mask = UINT32_MAX;
	f_config.single_filter = false;

	// Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
		printf("CAN Driver installed\n");
	} else {
		printf("Failed to install driver\n");
		return;
	}

	// Start TWAI driver
	if (twai_start() == ESP_OK) {
		printf("CAN Driver started\n");
	} else {
		printf("Failed to start driver\n");
		return;
	}
	//last_new_motor_time = millis();
}

void can_if::loop()
{
    #if 1
	
	while(1)
	{
		twai_message_t message;
		auto err = twai_receive(&message, 0);
		if (err == ESP_OK)
		{
			OnReceive(message);
		}
		else
		{
			break;
		}
		
	}
	
	#else
	CanInstance->Loop();
	#endif


    #if 0
	if (last_new_motor_time + 6000 < millis() && !foc_inited)
	{
		printf("Starting FOC...\n");
		for (auto &&i : motors)
		{
			auto message = RobotCAN::PacketType::INIT_FOC;
			can_send(i.discriminator, &message,  sizeof(message));
		}
		foc_inited = true;
	}
	//not connected, or button no longer held
	if (true)
	{
		if (foc_started != -1)
		{
			for (auto &&i : motors)
			{
				auto message = RobotCAN::PacketType::STOP_ALL;
				can_send(i.discriminator, &message,  sizeof(message));
			}
			foc_started = -1;
		}
	}
	else
	{
		//not started, enroll button, set mode
		if (foc_started == -1)
		{
			for (auto &&i : motors)
			{
				auto message = RobotCAN::PacketType::ENABLE_FOC;
				can_send(i.discriminator, &message,  sizeof(message));
			}
			std::sort(motors.begin(), motors.end(), [](const motor &a, const motor &b){
				return a.discriminator < b.discriminator;
			});
			printf("Motors enabled!\n");
			RobotCAN::Motor_mode_packet::mode_select type;
			if (0)
			{
				type = RobotCAN::Motor_mode_packet::mode_select::torque;
				foc_started = 0;
			}
			else if (0)
			{
				type = RobotCAN::Motor_mode_packet::mode_select::velocity;
				foc_started = 1;
			}
			
			for (auto &&i : motors)
			{
				RobotCAN::Motor_mode_packet message;
				message.mode = type;
				can_send(i.discriminator, &message,  sizeof(message));
			}
		}
		if (foc_started != -1)
		{
			float x_cons = 0;
			float y_cons = 0;
			float r_cons = 0;
			for (size_t i = 0; i < motors.size(); i++)
			{
				RobotCAN::Motor_target_packet message;
				float motor_angle = i*PI*2.f/3.f;
				message.target = x_cons * cosf(motor_angle) + y_cons*sinf(motor_angle) + r_cons;
				message.target *= 5;
				can_send(motors[i].discriminator, &message, sizeof(message));
			}
			//printf("x %f, y %f, r %f\n", x_cons, y_cons, r_cons);
		}
		
	}
    #endif
}

void can_if::register_callback(RobotCAN::PacketType type, OnReceiveCallback function, uint8_t min_length)
{
    CallbackMap[type] = {function, min_length};
}