#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }

    std::string iface = argv[1];

    std::string pipeline =
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=" + iface +
        " ! queue"
        " ! application/x-rtp, media=video, encoding-name=H264"
        " ! rtph264depay"
        " ! h264parse"
        " ! avdec_h264"
        " ! videoconvert"
        " ! video/x-raw,format=BGR"
        " ! appsink drop=1 sync=false";

    std::cout << "[INFO] Opening GStreamer pipeline:\n" << pipeline << std::endl;

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cerr << "[ERROR] Failed to open video stream with GStreamer." << std::endl;
        return -1;
    }

    cv::QRCodeDetector qrDetector;
    std::string lastQrText;
    uint64_t frameCount = 0;
    auto lastStatus = std::chrono::steady_clock::now();

    std::cout << "[INFO] Headless QR detection started. Press Ctrl+C to quit." << std::endl;

    while (true)
    {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty())
        {
            std::cerr << "[WARN] Failed to read frame." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        frameCount++;

        cv::Mat points;
        std::string qrText = qrDetector.detectAndDecode(frame, points);

        if (!qrText.empty())
        {
            std::cout << "[QR DETECTED] " << qrText << std::endl;

            if (!points.empty() && points.total() >= 4)
            {
                for (int i = 0; i < 4; i++)
                {
                    cv::Point p1(points.at<float>(i, 0), points.at<float>(i, 1));
                    cv::Point p2(points.at<float>((i + 1) % 4, 0), points.at<float>((i + 1) % 4, 1));
                    cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
                }
            }

            cv::putText(frame, qrText, cv::Point(30, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            if (qrText != lastQrText)
            {
                auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
                std::string filename = "/tmp/qr_detected_" + std::to_string(timestamp) + ".jpg";
                cv::imwrite(filename, frame);
                std::cout << "[INFO] Saved frame to " << filename << std::endl;
                lastQrText = qrText;
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatus).count() >= 2)
        {
            std::cout << "[INFO] Stream active, processed frames: " << frameCount << std::endl;
            lastStatus = now;
        }
    }

    cap.release();
    return 0;
}