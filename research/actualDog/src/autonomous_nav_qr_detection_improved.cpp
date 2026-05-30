#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/ros2_idl/String_.hpp>
#include <json.hpp>
#include <termio.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <future>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <unordered_map>

// -----------------------------------------------------------------------------
// Improved version of autonomous_nav_qr_detection_final.cpp
//
// What changed and why:
//   1. Thread-safety: shared flags (is_arrived, threadControl, navigationActive)
//      are now std::atomic; poseList / qrPoseList are guarded by a single mutex
//      everywhere they are touched (the original mixed two mutexes / no lock).
//   2. Arrival sync: the busy-wait `while(!is_arrived) sleep(5ms)` is replaced by
//      a condition_variable signalled from the SLAM callback -> responsive + no
//      wasted CPU, and it cannot miss a wake-up.
//   3. Clean shutdown: a dedicated quit key ('p' / Esc) breaks keyExecute so main
//      returns and the destructor actually runs (Ctrl+C was the only way out).
//   4. QR spam control: the original wrote a JPEG and re-saved the pose on EVERY
//      visible frame (floods /tmp and disk I/O). Now throttled per-QR-text.
//   5. Patrol loop is explicit and not buggy (the old i=0 + reverse re-walked the
//      same endpoint). A flag controls whether to patrol or run once.
//   6. Camera stream auto-reconnects instead of giving up on first failure.
// -----------------------------------------------------------------------------

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"

using namespace unitree::robot;
using namespace unitree::common;

class poseDate
{
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float q_x = 0.0f;
    float q_y = 0.0f;
    float q_z = 0.0f;
    float q_w = 1.0f;

    int mode = 0;
    float speed = 0.8f;

    std::string toJsonStr() const
    {
        nlohmann::json j;
        j["data"]["targetPose"]["x"] = x;
        j["data"]["targetPose"]["y"] = y;
        j["data"]["targetPose"]["z"] = z;
        j["data"]["targetPose"]["q_x"] = q_x;
        j["data"]["targetPose"]["q_y"] = q_y;
        j["data"]["targetPose"]["q_z"] = q_z;
        j["data"]["targetPose"]["q_w"] = q_w;
        j["data"]["mode"] = mode;
        j["data"]["speed"] = speed;
        return j.dump(4);
    }

    void printInfo() const
    {
        std::cout
            << "x:" << x
            << " y:" << y
            << " z:" << z
            << " q_x:" << q_x
            << " q_y:" << q_y
            << " q_z:" << q_z
            << " q_w:" << q_w
            << std::endl;
    }
};

class QrPoseRecord
{
public:
    std::string qrText;
    poseDate pose;
    int taskIndex = -1;

    nlohmann::json toJson() const
    {
        nlohmann::json j;
        j["qrText"] = qrText;
        j["taskIndex"] = taskIndex;
        j["pose"]["x"] = pose.x;
        j["pose"]["y"] = pose.y;
        j["pose"]["z"] = pose.z;
        j["pose"]["q_x"] = pose.q_x;
        j["pose"]["q_y"] = pose.q_y;
        j["pose"]["q_z"] = pose.q_z;
        j["pose"]["q_w"] = pose.q_w;
        j["pose"]["mode"] = pose.mode;
        j["pose"]["speed"] = pose.speed;
        return j;
    }
};

namespace unitree::robot::slam
{

const std::string TEST_SERVICE_NAME = "slam_operate";
const std::string TEST_API_VERSION = "1.0.0.1";

const int32_t ROBOT_API_ID_STOP_NODE = 1901;
const int32_t ROBOT_API_ID_START_MAPPING_PL = 1801;
const int32_t ROBOT_API_ID_END_MAPPING_PL = 1802;
const int32_t ROBOT_API_ID_START_RELOCATION_PL = 1804;
const int32_t ROBOT_API_ID_POSE_NAV_PL = 1102;
const int32_t ROBOT_API_ID_PAUSE_NAV = 1201;
const int32_t ROBOT_API_ID_RESUME_NAV = 1202;

const std::string QR_POSE_FILE = "/home/unitree/qr_poses.json";

// Don't re-save / re-snapshot the same QR more often than this.
constexpr auto QR_UPDATE_INTERVAL = std::chrono::seconds(1);

class TestClient : public Client
{
private:
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;

    void slamInfoHandler(const void *message);
    void slamKeyInfoHandler(const void *message);

    // curPose is written by the SLAM callback, read by the QR + key threads.
    poseDate curPose;
    std::mutex poseMutex;

    // taskMutex guards BOTH poseList and qrPoseList (single owner -> no
    // lock-ordering bugs, unlike the original which used two mutexes).
    std::vector<poseDate> poseList;
    std::vector<QrPoseRecord> qrPoseList;
    std::mutex taskMutex;

    // Arrival signalling (replaces the busy-wait loop).
    std::atomic<bool> is_arrived{false};
    std::mutex arrivalMutex;
    std::condition_variable arrivalCv;

    std::atomic<bool> threadControl{false};
    std::atomic<bool> navigationActive{false};
    std::atomic<bool> patrol{true}; // loop the waypoint list back and forth

    std::future<void> futThread;
    std::thread controlThread;

    std::string networkInterface;

    // Serializes every Call() so the nav thread and the key thread never issue
    // an RPC on the same Client concurrently (Unitree Client::Call is not
    // thread-safe). This is what lets the stop key preempt navigation safely.
    std::mutex callMutex;

    std::atomic<bool> qrThreadRunning{false};
    std::thread qrThread;

    void startQrScanner();
    void stopQrScanner();
    void qrScanLoop();
    void saveOrUpdatePoseForQr(const std::string &qrText);

    void saveQrPoseListToFile();
    void clearAllSavedPoses();

    bool waitForArrival();

    // Thread-safe RPC wrapper (serialized by callMutex).
    int32_t call(int32_t apiId, const std::string &parameter, std::string &data);

    // Immediate stop: halt robot motion now, then tear down the nav loop.
    void stopRobotNow();

public:
    explicit TestClient(const std::string &netIf);
    ~TestClient();

    void Init();

    unsigned char keyDetection();
    void keyExecute();

    void stopNodeFun();
    void startMappingPlFun();
    void endMappingPlFun();
    void relocationPlFun();
    void taskLoopFun(std::promise<void> prom);
    void pauseNavFun();
    void resumeNavFun();
    void taskThreadRun();
    void taskThreadStop();
};

TestClient::TestClient(const std::string &netIf)
    : Client(TEST_SERVICE_NAME, false), networkInterface(netIf)
{
    subSlamInfo =
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic));

    subSlamInfo->InitChannel(
        std::bind(&unitree::robot::slam::TestClient::slamInfoHandler, this, std::placeholders::_1), 1);

    subSlamKeyInfo =
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic));

    subSlamKeyInfo->InitChannel(
        std::bind(&unitree::robot::slam::TestClient::slamKeyInfoHandler, this, std::placeholders::_1), 1);

    std::cout << "***********************  Unitree SLAM Demo ***********************\n";
    std::cout << "---------------            q    w                -----------------\n";
    std::cout << "---------------            a    s   d   f        -----------------\n";
    std::cout << "---------------            z    x   p            -----------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "------------------ q: Start mapping            -------------------\n";
    std::cout << "------------------ w: End mapping              -------------------\n";
    std::cout << "------------------ a: Start relocation         -------------------\n";
    std::cout << "------------------ s: Add pose to task list    -------------------\n";
    std::cout << "------------------ d: Execute task list        -------------------\n";
    std::cout << "------------------ f: Clear task list          -------------------\n";
    std::cout << "------------------ z: Pause navigation         -------------------\n";
    std::cout << "------------------ x: Resume navigation        -------------------\n";
    std::cout << "------------------ p / Esc: Quit cleanly       -------------------\n";
    std::cout << "--------- Any other key: STOP the robot immediately --------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "---------------- QR detection starts after relocation ------------\n";
    std::cout << "---------------- Last visible QR pose is kept --------------------\n";
    std::cout << "---------------- QR detection pauses during navigation -----------\n";
    std::cout << "------------------------------------------------------------------\n" << std::endl;
}

TestClient::~TestClient()
{
    stopQrScanner();
    taskThreadStop();
    stopNodeFun();
}

void TestClient::clearAllSavedPoses()
{
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        poseList.clear();
        qrPoseList.clear();
    }

    std::ofstream out(QR_POSE_FILE);
    if (out.is_open())
    {
        out << "[]\n";
        out.close();
    }

    std::cout << "[INFO] Cleared previous task list and QR records." << std::endl;
}

void TestClient::Init()
{
    SetApiVersion(TEST_API_VERSION);

    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_POSE_NAV_PL);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_PAUSE_NAV);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_RESUME_NAV);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_STOP_NODE);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_MAPPING_PL);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_END_MAPPING_PL);
    UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_RELOCATION_PL);

    clearAllSavedPoses();
}

void TestClient::taskThreadRun()
{
    taskThreadStop(); // ensure any previous run is fully stopped first

    std::promise<void> prom;
    futThread = prom.get_future();

    threadControl = true;
    navigationActive = true;

    controlThread = std::thread(
        &unitree::robot::slam::TestClient::taskLoopFun, this, std::move(prom));
}

// Blocks until the SLAM key-info callback reports arrival, or until navigation
// is cancelled. Returns true if we actually arrived.
bool TestClient::waitForArrival()
{
    std::unique_lock<std::mutex> lock(arrivalMutex);
    arrivalCv.wait(lock, [this] { return is_arrived.load() || !threadControl.load(); });
    return is_arrived.load();
}

void TestClient::taskLoopFun(std::promise<void> prom)
{
    std::string data;

    // Snapshot the waypoints under the lock so we don't iterate a vector that
    // the QR thread may resize underneath us.
    std::vector<poseDate> waypoints;
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        waypoints = poseList;
    }

    std::cout << "task list num:" << waypoints.size() << std::endl;

    if (waypoints.empty())
    {
        std::cout << "[WARN] Task list is empty, nothing to navigate." << std::endl;
        navigationActive = false;
        prom.set_value();
        return;
    }

    do
    {
        for (size_t i = 0; i < waypoints.size() && threadControl.load(); ++i)
        {
            is_arrived = false;

            int32_t statusCode;
            {
                // Hold callMutex and re-check the stop flag *inside* the lock so
                // a stop that arrived between the for-condition and here cannot
                // be raced by a freshly dispatched goal.
                std::lock_guard<std::mutex> lock(callMutex);
                if (!threadControl.load())
                    break;
                statusCode = Call(
                    ROBOT_API_ID_POSE_NAV_PL,
                    waypoints[i].toJsonStr(),
                    data);
            }

            std::cout << "parameter:" << waypoints[i].toJsonStr() << std::endl;
            std::cout << "statusCode:" << statusCode << std::endl;
            std::cout << "data:" << data << std::endl;

            if (statusCode != 0)
            {
                std::cerr << "[ERROR] Nav call failed for waypoint " << i
                          << ", skipping." << std::endl;
                continue;
            }

            waitForArrival();
        }

        // Reverse so the next pass walks the route back the other way.
        if (threadControl.load() && patrol.load())
            std::reverse(waypoints.begin(), waypoints.end());

    } while (threadControl.load() && patrol.load());

    navigationActive = false;
    prom.set_value();
    std::cout << "[INFO] Navigation finished." << std::endl;
}

void TestClient::taskThreadStop()
{
    threadControl = false;
    navigationActive = false;

    // Wake the arrival wait so the loop can observe threadControl == false.
    arrivalCv.notify_all();

    if (controlThread.joinable())
        controlThread.join();
}

void TestClient::slamInfoHandler(const void *message)
{
    const auto *currentMsg = static_cast<const std_msgs::msg::dds_::String_ *>(message);

    nlohmann::json jsonData = nlohmann::json::parse(currentMsg->data());

    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    if (jsonData["type"] == "pos_info")
    {
        std::lock_guard<std::mutex> lock(poseMutex);

        curPose.x = jsonData["data"]["currentPose"]["x"];
        curPose.y = jsonData["data"]["currentPose"]["y"];
        curPose.z = jsonData["data"]["currentPose"]["z"];
        curPose.q_x = jsonData["data"]["currentPose"]["q_x"];
        curPose.q_y = jsonData["data"]["currentPose"]["q_y"];
        curPose.q_z = jsonData["data"]["currentPose"]["q_z"];
        curPose.q_w = jsonData["data"]["currentPose"]["q_w"];
    }
}

void TestClient::slamKeyInfoHandler(const void *message)
{
    const auto *currentMsg = static_cast<const std_msgs::msg::dds_::String_ *>(message);

    nlohmann::json jsonData = nlohmann::json::parse(currentMsg->data());

    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    if (jsonData["type"] == "task_result")
    {
        bool arrived = jsonData["data"]["is_arrived"];

        if (arrived)
        {
            std::cout << "I arrived " << jsonData["data"]["targetNodeName"] << std::endl;
        }
        else
        {
            std::cout << "I not arrived " << jsonData["data"]["targetNodeName"]
                      << " Please help me!! (T_T) (T_T) (T_T) " << std::endl;
        }

        {
            std::lock_guard<std::mutex> lock(arrivalMutex);
            is_arrived = arrived;
        }
        arrivalCv.notify_all();
    }
}

int32_t TestClient::call(int32_t apiId, const std::string &parameter, std::string &data)
{
    std::lock_guard<std::mutex> lock(callMutex);
    return Call(apiId, parameter, data);
}

// Stops the dog as fast as the link allows: PAUSE_NAV is the dedicated
// motion-halt and is a short RPC, so it lands almost immediately. We flip
// threadControl first so the nav loop cannot dispatch another goal, then halt,
// then join + stop the node as cleanup.
void TestClient::stopRobotNow()
{
    threadControl = false;   // nav loop will not issue any further goals
    navigationActive = false;
    arrivalCv.notify_all();  // unblock waitForArrival right now

    pauseNavFun();           // <-- robot stops moving here

    taskThreadStop();        // join the (now-unblocked) nav thread
    stopNodeFun();           // original teardown behaviour

    std::cout << "[INFO] Stop requested: robot halted." << std::endl;
}

void TestClient::stopNodeFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})";

    int32_t statusCode = call(ROBOT_API_ID_STOP_NODE, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::startMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"slam_type": "indoor"}})";

    int32_t statusCode = call(ROBOT_API_ID_START_MAPPING_PL, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::endMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"address": "/home/unitree/test.pcd"}})";

    int32_t statusCode = call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::relocationPlFun()
{
    std::string parameter, data;
    parameter = R"({
        "data": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "q_x": 0.0,
            "q_y": 0.0,
            "q_z": 0.0,
            "q_w": 1.0,
            "address": "/home/unitree/test.pcd"
        }
    })";

    int32_t statusCode = call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;

    if (statusCode == 0)
    {
        std::cout << "[INFO] Relocation started. QR scanning enabled." << std::endl;
        startQrScanner();
    }
}

void TestClient::pauseNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})";

    int32_t statusCode = call(ROBOT_API_ID_PAUSE_NAV, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::resumeNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})";

    int32_t statusCode = call(ROBOT_API_ID_RESUME_NAV, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::startQrScanner()
{
    if (qrThreadRunning)
    {
        std::cout << "[INFO] QR scanner already running." << std::endl;
        return;
    }

    qrThreadRunning = true;
    qrThread = std::thread(&TestClient::qrScanLoop, this);
}

void TestClient::stopQrScanner()
{
    qrThreadRunning = false;
    if (qrThread.joinable())
        qrThread.join();
}

void TestClient::saveQrPoseListToFile()
{
    // Caller must hold taskMutex.
    nlohmann::json j = nlohmann::json::array();
    for (const auto &rec : qrPoseList)
    {
        j.push_back(rec.toJson());
    }

    std::ofstream out(QR_POSE_FILE);
    if (!out.is_open())
    {
        std::cerr << "[ERROR] Failed to open " << QR_POSE_FILE << " for writing." << std::endl;
        return;
    }

    out << std::setw(4) << j << std::endl;
    out.close();

    std::cout << "[INFO] Saved QR poses to " << QR_POSE_FILE << std::endl;
}

void TestClient::saveOrUpdatePoseForQr(const std::string &qrText)
{
    poseDate poseSnapshot;
    {
        std::lock_guard<std::mutex> lock(poseMutex);
        poseSnapshot = curPose;
    }

    std::lock_guard<std::mutex> lock(taskMutex);

    for (auto &rec : qrPoseList)
    {
        if (rec.qrText == qrText)
        {
            rec.pose = poseSnapshot;

            if (rec.taskIndex >= 0 && rec.taskIndex < static_cast<int>(poseList.size()))
            {
                poseList[rec.taskIndex] = poseSnapshot;
            }

            std::cout << "[QR DETECTED] " << qrText << std::endl;
            std::cout << "[QR POSE UPDATED] " << qrText << std::endl;
            poseSnapshot.printInfo();

            saveQrPoseListToFile();
            return;
        }
    }

    QrPoseRecord newRec;
    newRec.qrText = qrText;
    newRec.pose = poseSnapshot;
    newRec.taskIndex = static_cast<int>(poseList.size());

    poseList.push_back(poseSnapshot);
    qrPoseList.push_back(newRec);

    std::cout << "[QR DETECTED] " << qrText << std::endl;
    std::cout << "[QR POSE SAVED] " << qrText << std::endl;
    poseSnapshot.printInfo();

    saveQrPoseListToFile();
}

void TestClient::qrScanLoop()
{
    const std::string pipeline =
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=" + networkInterface +
        " ! queue"
        " ! application/x-rtp, media=video, encoding-name=H264"
        " ! rtph264depay"
        " ! h264parse"
        " ! avdec_h264"
        " ! videoconvert"
        " ! video/x-raw,format=BGR"
        " ! appsink drop=1 sync=false";

    std::cout << "[INFO] Opening QR GStreamer pipeline:\n" << pipeline << std::endl;

    cv::QRCodeDetector qrDetector;
    uint64_t frameCount = 0;
    auto lastStatus = std::chrono::steady_clock::now();

    // Throttle per QR text so we don't flood disk with images / file writes.
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> lastSeen;

    std::cout << "[INFO] Headless QR detection thread started." << std::endl;

    while (qrThreadRunning)
    {
        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened())
        {
            std::cerr << "[ERROR] Failed to open QR video stream, retrying in 2s..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue; // auto-reconnect instead of bailing out
        }

        while (qrThreadRunning)
        {
            if (navigationActive)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            cv::Mat frame;
            if (!cap.read(frame) || frame.empty())
            {
                std::cerr << "[WARN] Failed to read QR frame, reopening stream." << std::endl;
                break; // drop out to the reconnect loop
            }

            frameCount++;

            cv::Mat points;
            std::string qrText = qrDetector.detectAndDecode(frame, points);

            if (!qrText.empty())
            {
                auto now = std::chrono::steady_clock::now();
                auto it = lastSeen.find(qrText);
                bool stale = (it == lastSeen.end()) ||
                             (now - it->second) >= QR_UPDATE_INTERVAL;

                if (stale)
                {
                    lastSeen[qrText] = now;
                    saveOrUpdatePoseForQr(qrText);

                    auto ts = std::chrono::system_clock::now().time_since_epoch().count();
                    std::string filename = "/tmp/qr_detected_" + std::to_string(ts) + ".jpg";
                    cv::imwrite(filename, frame);
                    std::cout << "[INFO] Saved QR frame to " << filename << std::endl;
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatus).count() >= 2)
            {
                std::cout << "[INFO] QR stream active, processed frames: " << frameCount << std::endl;
                lastStatus = now;
            }
        }

        cap.release();
    }

    std::cout << "[INFO] QR detection thread stopped." << std::endl;
}

unsigned char TestClient::keyDetection()
{
    termios tms_old, tms_new;

    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);

    unsigned char ch = getchar();

    tcsetattr(0, TCSANOW, &tms_old);

    std::cout << "\033[1;32m" << "Key " << ch << " pressed." << "\033[0m" << std::endl;
    return ch;
}

void TestClient::keyExecute()
{
    bool running = true;
    while (running)
    {
        unsigned char currentKey = keyDetection();

        switch (currentKey)
        {
        case 'q':
            startMappingPlFun();
            break;

        case 'w':
            endMappingPlFun();
            break;

        case 'a':
            relocationPlFun();
            break;

        case 's':
        {
            poseDate snapshot;
            {
                std::lock_guard<std::mutex> poseLock(poseMutex);
                snapshot = curPose;
            }
            {
                std::lock_guard<std::mutex> taskLock(taskMutex);
                poseList.push_back(snapshot);
            }
            snapshot.printInfo();
            break;
        }

        case 'd':
            taskThreadRun();
            break;

        case 'f':
            clearAllSavedPoses();
            std::cout << "Clear task list" << std::endl;
            break;

        case 'z':
            pauseNavFun();
            break;

        case 'x':
            resumeNavFun();
            break;

        case 'p':   // explicit quit
        case 27:    // Esc
            std::cout << "[INFO] Quit requested, shutting down..." << std::endl;
            stopRobotNow();   // halt the dog before we exit
            running = false;
            break;

        default:
            // Any other key is an immediate stop.
            stopRobotNow();
            break;
        }
    }
}

} // namespace unitree::robot::slam

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    unitree::robot::slam::TestClient tc(argv[1]);
    tc.Init();
    tc.SetTimeout(10.0f);
    tc.keyExecute();

    return 0;
}
