#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <iostream>
#include <cmath>
#include <unistd.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <networkInterface>\n";
        return -1;
    }

    // Initialize Unitree interface
    std::string interface_name = argv[1];
    unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

    unitree::robot::go2::SportClient client;
    client.SetTimeout(10.0f);
    client.Init();

    // Hardcoded target position near QR code
    float target_x = -0.26506;
    float target_y = -0.129303;
    float current_x = 0.0;
    float current_y = 0.0;

    float kp = 0.4f;
    float threshold = 0.1f;

    std::cout << "[INFO] Navigating to saved QR position...\n";

    while (true) {
        float dx = target_x - current_x;
        float dy = target_y - current_y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance < threshold) {
            std::cout << "[INFO] Reached target.\n";
            break;
        }

        float angle = std::atan2(dy, dx);
        float vx = kp * std::cos(angle);
        float vy = kp * std::sin(angle);

        vx = std::max(-2.5f, std::min(3.8f, vx));
        vy = std::max(-1.0f, std::min(1.0f, vy));

        client.Move(vx, vy, 0.0f);
        usleep(100000);  // 100 ms

        current_x += vx * 0.1f;
        current_y += vy * 0.1f;
    }

    client.Move(0.0f, 0.0f, 0.0f);
    client.StopMove();

    std::cout << "[INFO] Done.\n";
    return 0;
}
