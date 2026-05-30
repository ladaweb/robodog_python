#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unistd.h>  // for usleep

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

bool saved = false;

void HighStateHandler(const void* message)
{
    if (saved) return;

    if (!std::filesystem::exists("/tmp/qr_detected.txt")) return;

    const auto* state = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

    std::ofstream out("/tmp/robot_position_on_qr.txt");
    if (out.is_open()) {
        out << state->position()[0] << " "
            << state->position()[1] << " "
            << state->position()[2] << " "
            << state->imu_state().quaternion()[0] << " "
            << state->imu_state().quaternion()[1] << " "
            << state->imu_state().quaternion()[2] << " "
            << state->imu_state().quaternion()[3] << std::endl;

        out.close();
        saved = true;
        std::cout << "[POSITION SAVED]" << std::endl;
    } else {
        std::cerr << "[ERROR] Failed to open /tmp/robot_position_on_qr.txt for writing" << std::endl;
    }
}

int main()
{
    std::string networkInterface = "enp58s0";  // Update this if needed
    ChannelFactory::Instance()->Init(0, networkInterface);

    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> suber(TOPIC_HIGHSTATE);
    suber.InitChannel(HighStateHandler);

    std::cout << "[INFO] Waiting for QR detection and robot state..." << std::endl;

    while (!saved)
        usleep(20000);  // sleep for 20 ms

    return 0;
}
