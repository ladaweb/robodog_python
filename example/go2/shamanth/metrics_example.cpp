/**
 * metrics_example.cpp
 *
 * Reference snippets showing how to integrate MetricsLogger into your
 * existing SLAM + QR pipeline on the Unitree Go2 dock computer.
 *
 * You don't run this directly — copy the relevant snippets into your
 * real code, uncomment the Unitree/OpenCV lines, and remove placeholders.
 */

#include "metrics_logger.hpp"
#include <cmath>

// Your existing includes (uncomment on the robot):
// #include <unitree/robot/go2/sport/sport_client.hpp>
// #include <unitree/robot/go2/video/video_client.hpp>
// #include <unitree/idl/go2/LowState_.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/objdetect.hpp>

// ============================================================================
// 1. BATTERY — call from your LowState subscriber, every ~30s
// ============================================================================
void example_battery(MetricsLogger& logger) {
    // Option A: fill BatteryInfo manually from LowState fields
    BatteryInfo bat;
    // bat.soc        = low_state.bms_state().soc();
    // bat.status     = low_state.bms_state().status();
    // bat.current_mA = low_state.bms_state().current();
    // bat.voltage_V  = low_state.power_v();
    // bat.temp1_C    = low_state.temperature_ntc1();
    // bat.temp2_C    = low_state.temperature_ntc2();
    // bat.cycle      = low_state.bms_state().cycle();
    bat.soc = 85; bat.voltage_V = 28.4f; bat.current_mA = -2100; // placeholders
    logger.logBattery(bat);

    // Option B: one-liner if your LowState type has the right accessors
    // logger.logBatteryFromState(low_state);
}

// ============================================================================
// 2. QR DETECTION — wrap your detectAndDecode call with timing
// ============================================================================
void example_qr(MetricsLogger& logger) {
    // cv::QRCodeDetector qr_detector;
    // cv::Mat frame;  // from VideoClient

    std::string qrText;
    double decode_ms = MetricsLogger::measureQRDecode([&]() {
        // qrText = qr_detector.detectAndDecode(frame);
        qrText = "QR_MARKER_A";  // placeholder
    });

    if (!qrText.empty()) {
        Pose3D pose;
        // pose.x = curPose.x;
        // pose.y = curPose.y;
        // pose.z = curPose.yaw;  // z field stores yaw for 2D nav
        pose.x = 2.5; pose.y = 1.2; pose.z = 0.0; // placeholders

        logger.logQRDetection(qrText, decode_ms, pose);

        // Your existing waypoint save:
        // poseList.push_back(curPose);
    }
}

// ============================================================================
// 3. NAVIGATION — log start, then log result when done
// ============================================================================
void example_nav(MetricsLogger& logger) {
    Pose3D target;
    target.x = 2.5; target.y = 1.2; target.z = 0.0;

    Pose3D cur;
    cur.x = 0.0; cur.y = 0.0; cur.z = 0.0;

    // Log nav start
    logger.logNavigation(target, cur, NavEvent::START);

    // --- Your existing nav call ---
    // poseDate targetPose;
    // targetPose.x = 2.5; targetPose.y = 1.2; targetPose.z = 0.0;
    // std::string response;
    // Call(ROBOT_API_ID_POSE_NAV_PL, targetPose.toJsonStr(), response);
    // ... wait for navigation to complete ...

    // Log nav result
    cur.x = 2.48; cur.y = 1.19;
    logger.logNavigation(target, cur, NavEvent::REACHED);
    // or NavEvent::ABORTED if it timed out
    // or NavEvent::OBSTACLE if obstacle was hit
}

// ============================================================================
// 4. PERIODIC LOGGING — in your main loop
// ============================================================================
void example_periodic(MetricsLogger& logger) {
    // Pose breadcrumb (every ~0.5s or whatever your loop rate is)
    Pose3D cur;
    // cur.x = curPose.x; cur.y = curPose.y; cur.z = curPose.yaw;
    cur.x = 1.0; cur.y = 0.5;
    logger.logSLAMPose(cur);

    // System health (every ~30s) — tracks CPU load, RAM, disk on the dock
    logger.logSystemHealth();
}

// ============================================================================
// 5. FULL EXPERIMENT SKELETON
// ============================================================================
int main() {
    MetricsLogger logger("cooley_lab_run_01", "/home/unitree/logs");

    // --- Battery baseline ---
    example_battery(logger);

    // --- QR scanning phase ---
    for (int i = 0; i < 3; i++) {
        example_qr(logger);
        example_periodic(logger);
    }

    // --- Navigation phase ---
    example_nav(logger);

    // --- Final battery + system snapshot ---
    example_battery(logger);
    logger.logSystemHealth();

    std::cout << "Done. Elapsed: " << logger.elapsedSeconds() << "s\n";
    return 0;
}