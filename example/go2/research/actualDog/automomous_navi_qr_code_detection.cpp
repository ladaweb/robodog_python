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
#include <unordered_set>
#include <vector>
#include <iostream>
#include <chrono>
#include <algorithm>

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"

using namespace unitree::robot;
using namespace unitree::common;

unsigned char currentKey;

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

class TestClient : public Client
{
private:
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
    ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;

    void slamInfoHandler(const void *message);
    void slamKeyInfoHandler(const void *message);

    poseDate curPose;
    std::vector<poseDate> poseList;
    std::vector<QrPoseRecord> qrPoseList;

    bool is_arrived = false;
    bool threadControl = false;

    std::future<void> futThread;
    std::promise<void> prom;
    std::thread controlThread;

    std::string networkInterface;

    std::atomic<bool> qrThreadRunning{false};
    std::thread qrThread;
    std::mutex poseMutex;
    std::mutex qrMutex;
    std::unordered_set<std::string> loggedQrCodes;

    void startQrScanner();
    void stopQrScanner();
    void qrScanLoop();
    void savePoseForQr(const std::string &qrText);

public:
    explicit TestClient(const std::string &netIf);
    ~TestClient();

    void Init();

    unsigned char keyDetection();
    unsigned char keyExecute();

    void stopNodeFun();
    void startMappingPlFun();
    void endMappingPlFun();
    void relocationPlFun();
    void taskLoopFun(std::promise<void> &prom);
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

    std::cout << "*********************** Unitree SLAM Demo ***********************\n";
    std::cout << "--------------- q w -----------------\n";
    std::cout << "--------------- a s d f -----------------\n";
    std::cout << "--------------- z x -----------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "------------------ q: Start mapping -------------------\n";
    std::cout << "------------------ w: End mapping -------------------\n";
    std::cout << "------------------ a: Start relocation -------------------\n";
    std::cout << "------------------ s: Add pose to task list -------------------\n";
    std::cout << "------------------ d: Execute task list -------------------\n";
    std::cout << "------------------ f: Clear task list -------------------\n";
    std::cout << "------------------ z: Pause navigation -------------------\n";
    std::cout << "------------------ x: Resume navigation -------------------\n";
    std::cout << "---------------- Press any other key to stop SLAM ----------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "--------------- Press 'Ctrl + C' to exit the program -------------\n";
    std::cout << "------------------------------------------------------------------\n";
    std::cout << "------------------------------------------------------------------\n" << std::endl;
}

TestClient::~TestClient()
{
    stopQrScanner();
    taskThreadStop();
    stopNodeFun();
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
}

void TestClient::taskThreadRun()
{
    taskThreadStop();
    prom = std::promise<void>();
    futThread = prom.get_future();
    controlThread = std::thread(&unitree::robot::slam::TestClient::taskLoopFun, this, std::ref(prom));
    controlThread.detach();
}

void TestClient::taskLoopFun(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;

    std::cout << "task list num:" << poseList.size() << std::endl;

    for (int i = 0; i < static_cast<int>(poseList.size()); i++)
    {
        is_arrived = false;

        int32_t statusCode = Call(
            ROBOT_API_ID_POSE_NAV_PL,
            poseList[i].toJsonStr(),
            data);

        std::cout << "parameter:" << poseList[i].toJsonStr() << std::endl;
        std::cout << "statusCode:" << statusCode << std::endl;
        std::cout << "data:" << data << std::endl;

        while (!is_arrived)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (!threadControl)
                break;
        }

        if (i == static_cast<int>(poseList.size()) - 1)
        {
            i = 0;
            std::reverse(poseList.begin(), poseList.end());
        }

        if (!threadControl)
            break;
    }

    prom.set_value();
}

void TestClient::taskThreadStop()
{
    threadControl = false;

    if (futThread.valid())
    {
        auto status = futThread.wait_for(std::chrono::milliseconds(0));
        if (status != std::future_status::ready)
            futThread.wait();
    }
}

void TestClient::slamInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg =
        *(std_msgs::msg::dds_::String_ *)message;

    nlohmann::json jsonData = nlohmann::json::parse(currentMsg.data());

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
    std_msgs::msg::dds_::String_ currentMsg =
        *(std_msgs::msg::dds_::String_ *)message;

    nlohmann::json jsonData = nlohmann::json::parse(currentMsg.data());

    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    if (jsonData["type"] == "task_result")
    {
        is_arrived = jsonData["data"]["is_arrived"];

        if (is_arrived)
        {
            std::cout << "I arrived " << jsonData["data"]["targetNodeName"] << std::endl;
        }
        else
        {
            std::cout << "I not arrived " << jsonData["data"]["targetNodeName"]
                      << " Please help me!! (T_T) (T_T) (T_T) " << std::endl;
        }
    }
}

void TestClient::stopNodeFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})";

    int32_t statusCode = Call(ROBOT_API_ID_STOP_NODE, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::startMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"slam_type": "indoor"}})";

    int32_t statusCode = Call(ROBOT_API_ID_START_MAPPING_PL, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::endMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"address": "/home/unitree/test.pcd"}})";

    int32_t statusCode = Call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);

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

    int32_t statusCode = Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);

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

    int32_t statusCode = Call(ROBOT_API_ID_PAUSE_NAV, parameter, data);

    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void TestClient::resumeNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})";

    int32_t statusCode = Call(ROBOT_API_ID_RESUME_NAV, parameter, data);

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

void TestClient::savePoseForQr(const std::string &qrText)
{
    {
        std::lock_guard<std::mutex> lock(qrMutex);
        if (loggedQrCodes.find(qrText) != loggedQrCodes.end())
            return;
        loggedQrCodes.insert(qrText);
    }

    poseDate poseSnapshot;
    {
        std::lock_guard<std::mutex> lock(poseMutex);
        poseSnapshot = curPose;
        poseList.push_back(poseSnapshot);
    }

    {
        std::lock_guard<std::mutex> lock(qrMutex);
        QrPoseRecord rec;
        rec.qrText = qrText;
        rec.pose = poseSnapshot;
        qrPoseList.push_back(rec);
    }

    std::cout << "[QR DETECTED] " << qrText << std::endl;
    std::cout << "[QR POSE SAVED] " << qrText << std::endl;
    poseSnapshot.printInfo();
}

void TestClient::qrScanLoop()
{
    std::string pipeline =
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

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cerr << "[ERROR] Failed to open QR video stream with GStreamer." << std::endl;
        qrThreadRunning = false;
        return;
    }

    cv::QRCodeDetector qrDetector;
    std::string activeQrText;
    int noQrFrames = 0;
    const int qrLostThreshold = 10;

    uint64_t frameCount = 0;
    auto lastStatus = std::chrono::steady_clock::now();

    std::cout << "[INFO] Headless QR detection thread started." << std::endl;

    while (qrThreadRunning)
    {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty())
        {
            std::cerr << "[WARN] Failed to read QR frame." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        frameCount++;

        cv::Mat points;
        std::string qrText = qrDetector.detectAndDecode(frame, points);

        if (!qrText.empty())
        {
            noQrFrames = 0;

            if (qrText != activeQrText)
            {
                activeQrText = qrText;
                savePoseForQr(qrText);

                auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
                std::string filename = "/tmp/qr_detected_" + std::to_string(timestamp) + ".jpg";
                cv::imwrite(filename, frame);
                std::cout << "[INFO] Saved QR frame to " << filename << std::endl;
            }
        }
        else
        {
            noQrFrames++;
            if (noQrFrames >= qrLostThreshold)
                activeQrText.clear();
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatus).count() >= 2)
        {
            std::cout << "[INFO] QR stream active, processed frames: " << frameCount << std::endl;
            lastStatus = now;
        }
    }

    cap.release();
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

unsigned char TestClient::keyExecute()
{
    unsigned char currentKey;

    while (true)
    {
        currentKey = keyDetection();

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
            std::lock_guard<std::mutex> lock(poseMutex);
            poseList.push_back(curPose);
            curPose.printInfo();
            break;
        }

        case 'd':
            taskThreadRun();
            break;

        case 'f':
            poseList.clear();
            std::cout << "Clear task list" << std::endl;
            break;

        case 'z':
            pauseNavFun();
            break;

        case 'x':
            resumeNavFun();
            break;

        default:
            taskThreadStop();
            stopNodeFun();
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