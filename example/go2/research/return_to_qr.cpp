#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/trajectory_client.hpp>
#include <iostream>
#include <fstream>

using namespace unitree::robot;

int main()
{
    // Initialize communication
    std::string networkInterface = "enp58s0";  // Replace with your actual interface if different
    ChannelFactory::Instance()->Init(0, networkInterface);

    Go2TrajectoryClient client;
    client.Init();

    // Load position and orientation from file
    std::ifstream in("/tmp/robot_position_on_qr.txt");
    if (!in.is_open()) {
        std::cerr << "[ERROR] Failed to open /tmp/robot_position_on_qr.txt" << std::endl;
        return 1;
    }

    double x, y, z, qx, qy, qz, qw;
    if (!(in >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        std::cerr << "[ERROR] Invalid format in /tmp/robot_position_on_qr.txt" << std::endl;
        return 1;
    }

    std::cout << "[INFO] Target position: "
              << x << ", " << y << ", " << z << std::endl;
    std::cout << "[INFO] Target orientation (quaternion): "
              << qx << ", " << qy << ", " << qz << ", " << qw << std::endl;

    // Create target pose
    Go2TrajectoryClient::Pose pose;
    pose.position[0] = x;
    pose.position[1] = y;
    pose.position[2] = z;
    pose.quaternion[0] = qx;
    pose.quaternion[1] = qy;
    pose.quaternion[2] = qz;
    pose.quaternion[3] = qw;

    // Send the navigation target
    client.SendTarget(pose);
    std::cout << "[INFO] Sent target to robot." << std::endl;

    return 0;
}
