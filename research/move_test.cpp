#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// High-level status topic, where rt indicates real-time and lf indicates low frequency
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

// Callback function to obtain motion status
void HighStateHandler(const void* message)
{
  unitree_go::msg::dds_::SportModeState_ state = *(unitree_go::msg::dds_::SportModeState_*)message;

  // Print the robot's position
  std::cout<<"position: "
           <<state.position()[0]<<", "
           <<state.position()[1]<<", "
           <<state.position()[2]<<std::endl;
  // Print the robot's posture quaternion (w,x,y,z)
  std::cout<<": "
           <<state.imu_state().quaternion()[0]<<", "
           <<state.imu_state().quaternion()[1]<<", "
           <<state.imu_state().quaternion()[2]<<", "
           <<state.imu_state().quaternion()[3]<<std::endl;
}


int main()
{
  // Initialize the SDK interface
  std::string networkInterface = "enp2s0"; // Network interface name connected to the robot
  unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);

  // Create a Subscriber
  ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> suber(TOPIC_HIGHSTATE);

  // Initialize Channel
  suber.InitChannel(HighStateHandler);

  while(1)
  {
    usleep(20000);
  }

  return 0;
}