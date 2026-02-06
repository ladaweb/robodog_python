#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

static Pose2D g_pose;
static std::atomic<bool> g_have_pose{false};

static inline double wrapPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static inline double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline double yawFromQuat(double qx, double qy, double qz, double qw) {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

static void HighStateHandler(const void* message) {
    const auto* s = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

    // Copy ONLY primitives out of DDS message (safe)
    const double x = s->position()[0];
    const double y = s->position()[1];

    const auto q = s->imu_state().quaternion();
    const double yaw = yawFromQuat(q[0], q[1], q[2], q[3]);

    g_pose.x = x;
    g_pose.y = y;
    g_pose.yaw = yaw;

    g_have_pose.store(true, std::memory_order_release);
}

int main(int argc, char** argv) {
    // Hardcoded goal (your example)
    const double x_goal = -0.26506;
    const double y_goal = -0.129303;
    const double yaw_goal = M_PI / 2.0; // 90 deg

    // Controller params
    const double pos_tol = 0.10;                  // 10 cm
    const double yaw_tol = 8.0 * M_PI / 180.0;    // ~8 deg

    const double kp_dist = 0.9;
    const double kp_yaw  = 1.6;

    const double vx_max   = 0.4;   // m/s (indoor)
    const double vyaw_max = 0.8;   // rad/s (indoor)

    const int LOOP_US = 20000; // 50 Hz

    std::string iface = (argc >= 2) ? argv[1] : "enp58s0";

    // DDS init
    ChannelFactory::Instance()->Init(0, iface);

    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub(TOPIC_HIGHSTATE);
    sub.InitChannel(HighStateHandler);

    std::cout << "[INFO] Waiting for robot state...\n";
    while (!g_have_pose.load(std::memory_order_acquire)) {
        usleep(LOOP_US);
    }

    // Sport client
    unitree::robot::go2::SportClient sport;
    sport.SetTimeout(10.0f);
    sport.Init();

    std::cout << "[INFO] Returning to goal (" << x_goal << ", " << y_goal << ")\n";

    while (true) {
        const double x = g_pose.x;
        const double y = g_pose.y;
        const double yaw = g_pose.yaw;

        const double dx = x_goal - x;
        const double dy = y_goal - y;
        const double dist = std::sqrt(dx*dx + dy*dy);

        // Reached position?
        if (dist < pos_tol) {
            const double yaw_err = wrapPi(yaw_goal - yaw);

            if (std::fabs(yaw_err) < yaw_tol) {
                sport.Move(0, 0, 0);
                std::cout << "[DONE] Reached goal within 10 cm + yaw aligned.\n";
                break;
            }

            const double vyaw = clampd(kp_yaw * yaw_err, -vyaw_max, vyaw_max);
            sport.Move(0, 0, (float)vyaw);
            usleep(LOOP_US);
            continue;
        }

        // Drive-to-goal
        const double desired_heading = std::atan2(dy, dx);
        const double heading_err = wrapPi(desired_heading - yaw);

        double vx = clampd(kp_dist * dist, 0.0, vx_max);
        double vyaw = clampd(kp_yaw * heading_err, -vyaw_max, vyaw_max);

        // slow down if facing too far away
        if (std::fabs(heading_err) > M_PI / 4.0) vx *= 0.2;

        sport.Move((float)vx, 0.0f, (float)vyaw);

        static int k = 0;
        if (++k % 10 == 0) {
            std::cout << "[DBG] dist=" << dist
                      << " heading_err=" << heading_err
                      << " cmd(vx,vyaw)=(" << vx << "," << vyaw << ")\n";
        }

        usleep(LOOP_US);
    }

    // Ensure stop
    sport.Move(0, 0, 0);
    return 0;
}
