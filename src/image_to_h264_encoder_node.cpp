#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/format.hpp"

#include <fstream>
#include <iostream>
using namespace std;

int _imageHeight = -1;
int _imageWidth = -1;

int _fps = 20;
cv::VideoWriter videoWriter;
string logFilePath = "/tmp/recorderLog.txt";
ofstream *logFile = nullptr;
int _currentFrame = 0;
int _decoderType = 0;

// Support for depth images
bool _isDepthImage = false;
float _maxDepth = 100.0;
cv::Mat *monoImg = nullptr;
cv::Mat rgbDepthImg;

// Define the basic Gstreamer pipeline
string _gstPipelineTemplate = "appsrc ! videoconvert ! %s bitrate=%d ! %s mp4mux ! filesink location=%s ";

// Define the default decoder values
string _encoder = "nvh264enc rc-mode=2";
int _bitrate = 10000;
string _parser = "h264parse !";
string _location = "/tmp/pipe.mp4";

// Publisher for the image header
ros::Publisher pub;

bool initialize(int w, int h, int decoderType = 0) {
    _imageHeight = h;
    _imageWidth = w;

    switch (decoderType) {
        case 0:
            // NVIDA nvh264enc
            ROS_INFO("Encoder set to nvh264enc");
            break;

        case 1:
            // avenc_mpeg4 fallback
            _encoder = "avenc_mpeg4";
            _parser = "";  // no parser needed
            ROS_INFO("Encoder set to avenc_mpeg4");
            break;

        default:
            // NVIDA nvh264enc
            ROS_WARN("Invalid encoder type passed, using default nvh264enc");
            break;
    }

    try {
        string _gstPipeline = boost::str(boost::format(_gstPipelineTemplate) % _encoder % _bitrate % _parser % _location);
        ROS_INFO("GSTP: %s", _gstPipeline.c_str());
        videoWriter.open(_gstPipeline, cv::CAP_GSTREAMER, 0, (double)_fps, cv::Size(_imageWidth, _imageHeight));
        logFile = new ofstream(logFilePath);

        *logFile << "Recording options: " << _encoder << " bitrate=" << _bitrate << "\n";

        return true;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return false;
    }
}

string getTimeStampString(ros::Time rosTime) {
    return boost::posix_time::to_iso_extended_string(rosTime.toBoost());
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    auto start = std::chrono::high_resolution_clock::now();
    // get some information from the header
    uint32_t _seq = msg->header.seq;
    ros::Time _stamp = msg->header.stamp;

    cv_bridge::CvImagePtr cv_ptr;
    if (_isDepthImage) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            if (monoImg == nullptr) {
                monoImg = new cv::Mat(cv_ptr->image.size(), CV_8UC1);
            }
            cv::convertScaleAbs(cv_ptr->image, *monoImg, 255.0 / _maxDepth, 0.0);
            cv::cvtColor(*monoImg, rgbDepthImg, cv::COLOR_GRAY2RGB);
        } catch (const std::exception &e) {
            ROS_ERROR("DepthImage: cv_bridge exception: %s", e.what());
            std::cerr << e.what() << '\n';
            return;
        }

    } else {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("RGBImage: cv_bridge exception: %s", e.what());
            return;
        }
    }

    // do we have to initialize?
    if (_imageHeight == -1) {
        assert(("Error during initialization", initialize(cv_ptr->image.cols, cv_ptr->image.rows, _decoderType)));
    }

    // write the image to the h264 file
    if (_isDepthImage) {
        videoWriter.write(rgbDepthImg);
    } else {
        videoWriter.write(cv_ptr->image);
    }

    // write to the log file
    *logFile << _currentFrame << " " << _seq << " " << getTimeStampString(_stamp) << "\n";

    _currentFrame += 1;
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    double _hz = 1.0 / elapsed.count();

    ROS_INFO("Capable of %f fps", _hz);
    pub.publish(msg->header);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_to_h264_encoder");
    ros::NodeHandle n("~");

    // get some parameters
    n.getParam("fps", _fps);
    n.getParam("logFilePath", logFilePath);
    n.getParam("decoderType", _decoderType);
    n.getParam("bitrate", _bitrate);
    n.getParam("targetLocation", _location);

    n.getParam("isDepthImage", _isDepthImage);
    n.getParam("maxDepth", _maxDepth);

    ros::Subscriber sub = n.subscribe("image", 1000, imageCallback);
    pub = n.advertise<std_msgs::Header>("header", 1000);

    ros::spin();

    if (logFile != nullptr) {
        logFile->close();
    }

    videoWriter.release();

    return 0;
}