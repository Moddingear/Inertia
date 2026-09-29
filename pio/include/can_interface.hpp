#pragma once

#include <driver/twai.h>
#include <CANRobot.hpp>

namespace can_if
{
    typedef std::function<bool(const twai_message_t)> OnReceiveCallback;
    void setup();
    void loop();
    void register_callback(RobotCAN::PacketType type, OnReceiveCallback function, uint8_t min_length = 0);
} // namespace can_if
