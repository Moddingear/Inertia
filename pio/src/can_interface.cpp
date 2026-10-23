
#include <Arduino.h>
#include <CANRobot.hpp>
#include <can_interface.hpp>
#include <map>
#ifdef ComplexCAN
#include <ComplexCAN_ESP32.h>
#else
#include <esp_twai.h>
#include <esp_twai_onchip.h>
#include <freertos/ringbuf.h>
#endif

#define CAN_TX GPIO_NUM_36
#define CAN_RX GPIO_NUM_35
#define CAN_SD GPIO_NUM_37



static std::map<RobotCAN::PacketType, std::pair<can_if::OnReceiveCallback, uint8_t>> CallbackMap;
ComplexCan_ESP32 CanInstance(CAN_TX, CAN_RX);

FilterDefinition filter[] ={
{ //accept only us
	.FilterIndex = 0,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_TO_RXFIFO0,
	.ExtendedID = false,
	.FilterID2 = RobotCAN::hub_mask_std,
},
{ //accept only us
	.FilterIndex = 0,
	.FilterType = CAN_FILTER_MASK,
	.FilterConfig = CAN_FILTER_TO_RXFIFO0,
	.ExtendedID = true,
	.FilterID2 = RobotCAN::hub_mask_extd,
},
};

void OnReceive(const ComplexCanRxHeader &rxHeader, const uint8_t *rxData, void* userData)
{
	if (rxHeader.DataLength < sizeof(RobotCAN::PacketType) && !rxHeader.ExtendedID)
	{
		printf("Received a message that is too short.\n");
		return;
	}
	RobotCAN::PacketType type;
	uint8_t actual_length;
	if (rxHeader.ExtendedID)
	{
		RobotCAN::ID_extended header = rxHeader.Identifier;
		type = header.packet_type;
		actual_length = rxHeader.DataLength;
	}
	else
	{
		type = static_cast<RobotCAN::PacketType>(rxData[0]);
		actual_length = rxHeader.DataLength-1;
	}
	auto entry = CallbackMap.find(type);
	if (entry == CallbackMap.end())
	{
		printf("Missing callback for packet type %d on CAN (identifier %#x)\n", type, rxHeader.Identifier);
		return;
	}
	if (actual_length < entry->second.second)
	{
		printf("Packet too small for %d on CAN (%d)\n", type, actual_length);
		return;
	}
	entry->second.first(rxHeader, &rxData[1-rxHeader.ExtendedID]);
}

void can_if::send(uint32_t id, void* data, uint8_t length, bool single_shot)
{
	CanInstance.SendMessage(reinterpret_cast<uint8_t*>(data), length, id, true);
}

void can_if::request(uint32_t id, uint8_t expected_length, bool single_shot)
{
	CanInstance.RequestMessage(expected_length, id, true);
}

//key is identifier, value is data
static std::map<uint8_t, std::vector<uint8_t>> fractionnal_packet_buffer;

bool OnFraction(const ComplexCanRxHeader &rxHeader, const uint8_t *rxData)
{
	if (!rxHeader.ExtendedID)
	{
		return false;
	}
	
	auto header = RobotCAN::ID_extended(rxHeader.Identifier);
	uint8_t key = header.motor_index;
	auto &entry = fractionnal_packet_buffer[key];
	if (header.packet_index == 0)
	{
		entry.resize(0);
		entry.reserve(rxData[0]*8-1);
	}
	for (size_t i = header.packet_index == 0 ? 1 : 0; i < rxHeader.DataLength; i++)
	{
		entry.push_back(rxData[i]);
	}
	if (entry.capacity() - entry.size() < 8)
	{
		ComplexCanRxHeader fake_frame = rxHeader;
		fake_frame.DataLength = entry.size()-1;
		header.packet_type = static_cast<RobotCAN::PacketType>(entry.at(0));
		fake_frame.Identifier = header;
		OnReceive(fake_frame, &entry.at(1), nullptr);
		fractionnal_packet_buffer.erase(key);
	}
	return true;
}

void can_if::setup()
{
	pinMode(CAN_SD, OUTPUT);
	pinMode(CAN_TX, OUTPUT);
	pinMode(CAN_RX, INPUT);
	digitalWrite(CAN_SD, LOW);

	#ifdef ComplexCAN
	CanInstance.Init(1000000, 5000000);
	filter[0].FilterID1 = RobotCAN::ID_standard(0, HUB_INDEX, true, false);
	filter[1].FilterID1 = RobotCAN::ID_extended(0, static_cast<RobotCAN::PacketType>(0), 0, HUB_INDEX, true, 0);
	for (size_t i = 0; i < sizeof(filter)/sizeof(filter[0]); i++)
	{
		auto status = CanInstance.ConfigFilter(&filter[i]);
		printf("Filter %d status %d (%#x # %#x)\n", i, status, filter[i].FilterID1, filter[i].FilterID2);
	}
	CanInstance.ConfigGlobalFilter();
	CanInstance.ActivateNotification(OnReceive, nullptr);
	CanInstance.Start();
	#else
	rx_queue = xRingbufferCreateNoSplit(sizeof(twai_listener_data_t), 32);
	if (rx_queue == NULL) {
        printf("Failed to create ring buffer\n");
    }

	twai_filter_config_t f_config;
	f_config.acceptance_code = (static_cast<uint32_t>(RobotCAN::IDOffset::prio_hub) << 16) | static_cast<uint32_t>(RobotCAN::IDOffset::to_hub);
	f_config.acceptance_mask = (static_cast<uint32_t>(RobotCAN::hub_mask) << 16) | RobotCAN::hub_mask;
	f_config.single_filter = false;

	twai_onchip_node_config_t node_config = {
		.io_cfg = {
			.tx = CAN_TX,
			.rx = CAN_RX,
			.quanta_clk_out = GPIO_NUM_NC,
			.bus_off_indicator = GPIO_NUM_NC,
		},
		.bit_timing = {
			.bitrate = 1000000,
		},
		.fail_retry_cnt = 3,
		.tx_queue_depth = 10,
	};
	ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &can_handle));

	twai_event_callbacks_t callbacks = {
		.on_tx_done = twai_tx_callback,
		.on_rx_done = twai_rx_callback,
		.on_state_change = twai_on_state_change_callback,
		.on_error = twai_on_error_callback,
	};
	ESP_ERROR_CHECK(twai_node_register_event_callbacks(can_handle, &callbacks, NULL));
	ESP_ERROR_CHECK(twai_node_enable(can_handle));
	#endif

	printf("CAN Driver installed\n");

	register_callback(RobotCAN::PacketType::FRACTION, OnFraction, 1);
}

void can_if::loop()
{
	#ifdef ComplexCAN
	CanInstance.Loop();
	#else
	for (;;)
	{
		size_t length;
		twai_listener_data_t *message = reinterpret_cast<twai_listener_data_t*>(xRingbufferReceive(rx_queue, &length, 0));
		if (message == nullptr)
		{
			break;
		}
		message->frame.buffer = message->data;
		OnReceive(message->frame);
		vRingbufferReturnItem(rx_queue, message);
	}
	#endif

	#if 0
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