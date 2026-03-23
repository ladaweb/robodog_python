#pragma once
/**
 * metrics_logger.hpp
 * 
 * Lightweight metrics logging for the AIMS Lab robodog radiation mapping project.
 * Designed to run on the Unitree Go2 dock computer (aarch64 Linux).
 *
 * Logs to CSV for easy post-processing. No external dependencies beyond
 * the C++ standard library and Unitree SDK2 headers.
 *
 * Usage:
 *   MetricsLogger logger("run_001");
 *   logger.logBattery(batteryInfo);
 *   logger.logQRDetection("QR_WAYPOINT_3", decode_ms, cur_pose);
 *   logger.logNavigation(target_pose, cur_pose, NavEvent::START);
 *   logger.logSLAMPose(cur_pose);
 *   logger.logSystemHealth();
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/statvfs.h>

// ---------------------------------------------------------------------------
// Pose struct (matches your poseDate but kept self-contained)
// ---------------------------------------------------------------------------
struct Pose3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;    // yaw for 2-D nav, or z for 3-D
};

// ---------------------------------------------------------------------------
// Battery snapshot (mirrors BmsState fields from LowState)
// ---------------------------------------------------------------------------
struct BatteryInfo {
    uint8_t  soc        = 0;     // State of charge 0-100 %
    uint8_t  status     = 0;     // 0 wakeup, 1 discharge, 2 charge ...
    int32_t  current_mA = 0;     // Instantaneous current (mA)
    float    voltage_V  = 0.0f;  // Pack voltage (from power_v)
    int8_t   temp1_C    = 0;     // NTC1 temperature
    int8_t   temp2_C    = 0;     // NTC2 temperature
    uint16_t cycle      = 0;     // Charge cycle count
};

// ---------------------------------------------------------------------------
// Navigation event tags
// ---------------------------------------------------------------------------
enum class NavEvent : uint8_t {
    START       = 0,   // Navigation command issued
    REACHED     = 1,   // Target pose reached
    ABORTED     = 2,   // Navigation aborted / timed out
    OBSTACLE    = 3,   // Obstacle avoidance triggered
};

static inline const char* navEventStr(NavEvent e) {
    switch (e) {
        case NavEvent::START:    return "START";
        case NavEvent::REACHED:  return "REACHED";
        case NavEvent::ABORTED:  return "ABORTED";
        case NavEvent::OBSTACLE: return "OBSTACLE";
    }
    return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// MetricsLogger
// ---------------------------------------------------------------------------
class MetricsLogger {
public:
    /**
     * @param run_id   A short identifier for this run (e.g. "run_001").
     * @param out_dir  Directory for log files. Created if missing.
     */
    explicit MetricsLogger(const std::string& run_id,
                           const std::string& out_dir = "./logs")
        : run_id_(run_id), out_dir_(out_dir)
    {
        // Ensure output directory exists
        // Create output directory
        std::string cmd = "mkdir -p " + out_dir_;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
        std::system(cmd.c_str());
#pragma GCC diagnostic pop

        t0_ = Clock::now();

        // Open individual CSV files with headers
        openFile(battery_file_,  "battery",
                 "elapsed_s,timestamp,soc_pct,status,current_mA,voltage_V,"
                 "temp1_C,temp2_C,cycle");

        openFile(qr_file_,       "qr",
                 "elapsed_s,timestamp,qr_text,decode_ms,pose_x,pose_y,pose_yaw");

        openFile(nav_file_,      "nav",
                 "elapsed_s,timestamp,event,target_x,target_y,target_yaw,"
                 "cur_x,cur_y,cur_yaw,dist_to_target");

        openFile(pose_file_,     "pose",
                 "elapsed_s,timestamp,x,y,yaw");

        openFile(system_file_,   "system",
                 "elapsed_s,timestamp,cpu_load_1m,mem_used_MB,mem_total_MB,"
                 "disk_avail_MB");

        std::cout << "[MetricsLogger] Logging to " << out_dir_ << "/"
                  << run_id_ << "_*.csv\n";
    }

    ~MetricsLogger() {
        std::lock_guard<std::mutex> lk(mtx_);
        battery_file_.close();
        qr_file_.close();
        nav_file_.close();
        pose_file_.close();
        system_file_.close();
    }

    // ===== Battery ==========================================================

    void logBattery(const BatteryInfo& b) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto [elapsed, ts] = now();
        battery_file_ << elapsed << "," << ts << ","
                      << static_cast<int>(b.soc) << ","
                      << static_cast<int>(b.status) << ","
                      << b.current_mA << ","
                      << b.voltage_V << ","
                      << static_cast<int>(b.temp1_C) << ","
                      << static_cast<int>(b.temp2_C) << ","
                      << b.cycle << "\n";
        battery_file_.flush();
    }

    /**
     * Convenience: pull battery info straight from a Unitree LowState.
     * Template avoids a hard #include on Unitree headers — any type that
     * exposes .bms_state().soc(), .power_v(), etc. will compile.
     */
    template <typename LowStateT>
    void logBatteryFromState(const LowStateT& ls) {
        BatteryInfo b;
        b.soc        = ls.bms_state().soc();
        b.status     = ls.bms_state().status();
        b.current_mA = ls.bms_state().current();
        b.voltage_V  = ls.power_v();
        b.temp1_C    = ls.temperature_ntc1();
        b.temp2_C    = ls.temperature_ntc2();
        b.cycle      = ls.bms_state().cycle();
        logBattery(b);
    }

    // ===== QR Detection =====================================================

    /**
     * Log a QR detection event.
     * @param qr_text    Decoded QR string
     * @param decode_ms  Time to decode (ms). Use measureQRDecode() helper.
     * @param pose       Robot's SLAM pose at detection time
     */
    void logQRDetection(const std::string& qr_text,
                        double decode_ms,
                        const Pose3D& pose)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto [elapsed, ts] = now();
        qr_file_ << elapsed << "," << ts << ","
                  << sanitize(qr_text) << ","
                  << decode_ms << ","
                  << pose.x << "," << pose.y << "," << pose.z << "\n";
        qr_file_.flush();
    }

    // ===== Navigation =======================================================

    void logNavigation(const Pose3D& target,
                       const Pose3D& current,
                       NavEvent event)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto [elapsed, ts] = now();
        double dist = std::sqrt(
            (target.x - current.x) * (target.x - current.x) +
            (target.y - current.y) * (target.y - current.y));

        nav_file_ << elapsed << "," << ts << ","
                  << navEventStr(event) << ","
                  << target.x << "," << target.y << "," << target.z << ","
                  << current.x << "," << current.y << "," << current.z << ","
                  << dist << "\n";
        nav_file_.flush();
    }

    // ===== SLAM Pose (periodic breadcrumb) ==================================

    void logSLAMPose(const Pose3D& pose) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto [elapsed, ts] = now();
        pose_file_ << elapsed << "," << ts << ","
                   << pose.x << "," << pose.y << "," << pose.z << "\n";
        pose_file_.flush();
    }

    // ===== System Health (CPU, RAM, Disk) ====================================

    void logSystemHealth() {
        std::lock_guard<std::mutex> lk(mtx_);
        auto [elapsed, ts] = now();

        // CPU load average (1 min)
        double load_1m = 0.0;
        std::ifstream loadavg("/proc/loadavg");
        if (loadavg.is_open()) {
            loadavg >> load_1m;
        }

        // Memory via /proc/meminfo
        double mem_total_MB = 0.0;
        double mem_used_MB  = 0.0;
        {
            std::ifstream memf("/proc/meminfo");
            std::string line;
            double mem_total_kb = 0, mem_avail_kb = 0;
            while (std::getline(memf, line)) {
                if (line.rfind("MemTotal:", 0) == 0)
                    std::sscanf(line.c_str(), "MemTotal: %lf kB", &mem_total_kb);
                else if (line.rfind("MemAvailable:", 0) == 0)
                    std::sscanf(line.c_str(), "MemAvailable: %lf kB", &mem_avail_kb);
            }
            mem_total_MB = mem_total_kb / 1024.0;
            mem_used_MB  = (mem_total_kb - mem_avail_kb) / 1024.0;
        }

        // Disk available on the logging partition
        struct statvfs sv{};
        statvfs(out_dir_.c_str(), &sv);
        double disk_avail_MB = static_cast<double>(sv.f_bavail) * sv.f_bsize
                               / (1024.0 * 1024.0);

        system_file_ << elapsed << "," << ts << ","
                     << load_1m << ","
                     << mem_used_MB << "," << mem_total_MB << ","
                     << disk_avail_MB << "\n";
        system_file_.flush();
    }

    // ===== Timing helpers ===================================================

    /** Elapsed seconds since logger was created. */
    double elapsedSeconds() const {
        return std::chrono::duration<double>(Clock::now() - t0_).count();
    }

    /**
     * Helper: time a QR decode call in milliseconds.
     *
     * Example:
     *   std::string text;
     *   double ms = MetricsLogger::measureQRDecode([&]() {
     *       text = qr_detector.detectAndDecode(frame);
     *   });
     *   if (!text.empty()) logger.logQRDetection(text, ms, pose);
     */
    template <typename Func>
    static double measureQRDecode(Func&& fn) {
        auto start = Clock::now();
        fn();
        auto end = Clock::now();
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

private:
    using Clock    = std::chrono::steady_clock;
    using SysClock = std::chrono::system_clock;

    std::string run_id_;
    std::string out_dir_;
    Clock::time_point t0_;
    std::mutex mtx_;

    std::ofstream battery_file_;
    std::ofstream qr_file_;
    std::ofstream nav_file_;
    std::ofstream pose_file_;
    std::ofstream system_file_;

    void openFile(std::ofstream& f,
                  const std::string& tag,
                  const std::string& header)
    {
        std::string path = out_dir_ + "/" + run_id_ + "_" + tag + ".csv";
        f.open(path, std::ios::out | std::ios::trunc);
        if (!f.is_open()) {
            std::cerr << "[MetricsLogger] ERROR: cannot open " << path << "\n";
            return;
        }
        f << header << "\n";
    }

    /** Return {elapsed_seconds, ISO-8601 wall-clock timestamp}. */
    std::pair<double, std::string> now() const {
        double elapsed = std::chrono::duration<double>(
            Clock::now() - t0_).count();

        auto wall = SysClock::now();
        auto tt   = SysClock::to_time_t(wall);
        auto ms   = std::chrono::duration_cast<std::chrono::milliseconds>(
                        wall.time_since_epoch()) % 1000;

        std::tm buf{};
        localtime_r(&tt, &buf);

        std::ostringstream oss;
        oss << std::put_time(&buf, "%Y-%m-%dT%H:%M:%S")
            << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return {elapsed, oss.str()};
    }

    /** Strip commas/newlines so CSV stays valid. */
    static std::string sanitize(const std::string& s) {
        std::string out = s;
        for (auto& c : out) {
            if (c == ',' || c == '\n' || c == '\r') c = '_';
        }
        return out;
    }
};
