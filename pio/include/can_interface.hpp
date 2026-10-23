#pragma once

#include <CANRobot.hpp>
#include <functional>
#include <ComplexCAN.h>

namespace can_if
{
    typedef std::function<bool(const ComplexCanRxHeader&, const uint8_t*)> OnReceiveCallback;
    void setup();
    void loop();
    void send(uint32_t id, void* data, uint8_t length, bool single_shot = false);
    void request(uint32_t id, uint8_t expected_length, bool single_shot = false);
    void register_callback(RobotCAN::PacketType type, OnReceiveCallback function, uint8_t min_length = 0);
} // namespace can_if
