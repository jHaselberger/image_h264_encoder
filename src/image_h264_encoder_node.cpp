#define OPENCVVERSION 4

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/format.hpp"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
using namespace std;

bool debugmode = false;

int _imageHeight = -1;
int _imageWidth = -1;

int _fps = 20;
cv::VideoWriter videoWriter;
//string logFilePath = "/tmp/recorderLog.txt";
ofstream *logFile = nullptr;
int _currentFrame = 0;
int _decoderType = 0;

string _debugPrefix = "";

// Support for gray and depth images
bool _isGrayImage = false;
bool _isDepthImage = false;
float _maxDepth = 100.0;
cv::Mat *monoImg = nullptr;
cv::Mat rgbImg;

// republish scaled image
bool _publishScaledImage = false;
float _imageScaleFactor = 0.25;
cv::Mat scaledImg;
cv_bridge::CvImage scaledImgBridge;
sensor_msgs::Image scaledImgMsg;

// Define the basic Gstreamer pipeline
string _gstPipelineTemplate = "appsrc ! videoconvert ! %s bitrate=%d ! %s mp4mux ! filesink location=%s ";

// Define the default decoder values
string _encoder = "nvh264enc rc-mode=2";
int _bitrate = 10000;
string _parser = "h264parse !";
string _location = "/tmp/pipe.mp4";

// Publisher for the image header
ros::Publisher pub;
ros::Publisher imagePub;

// for logging
auto start = std::chrono::high_resolution_clock::now();;

void _logDebug(std::string what, bool enable = debugmode) {
    if (enable) {
        std::cout << "\n\t-> " << what << std::endl;
    }
}

// found here: https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
bool _replace(std::string &str, const std::string &from, const std::string &to) {
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

// found here: https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
void _replaceAll(std::string &str, const std::string &from, const std::string &to) {
    if (from.empty())
        return;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();  // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

bool _initialize(int w, int h, std::string ts, int decoderType = 0) {
    _logDebug("call initialize");

    _imageHeight = h;
    _imageWidth = w;

    std::string _dot = ".";
    std::string timeStamp = ts;
    _replaceAll(timeStamp, ".", "-");

    std::string _locationTS = _location;
    _replace(_locationTS, ".", timeStamp.append(_dot));
    _replaceAll(_locationTS, ":", "-");

    std::string _logFilePathTS = _locationTS;
    _replace(_logFilePathTS, ".mp4", ".txt");

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
        string _gstPipeline = boost::str(boost::format(_gstPipelineTemplate) % _encoder % _bitrate % _parser % _locationTS);
        ROS_INFO("GSTP: %s", _gstPipeline.c_str());

#if OPENCVVERSION == 3
        videoWriter.open(_gstPipeline, 0, (double)_fps, cv::Size(_imageWidth, _imageHeight));
#else
        videoWriter.open(_gstPipeline, cv::CAP_GSTREAMER, 0, (double)_fps, cv::Size(_imageWidth, _imageHeight));
#endif

        logFile = new ofstream(_logFilePathTS);

        *logFile << "Recording options: " << _encoder << " bitrate=" << _bitrate << "\n";
        *logFile << "fileFrameNumber"
                 << " "
                 << "seq"
                 << " "
                 << "timeStamp"
                 << "\n";

        return true;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return false;
    }
}

string getTimeStampString(ros::Time rosTime, int hoursOffset = 2) {
    rosTime = rosTime + ros::Duration(hoursOffset * 60 * 60);
    return boost::posix_time::to_iso_extended_string(rosTime.toBoost());
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    _logDebug("Entering image callback");
    if (debugmode) {
        start = std::chrono::high_resolution_clock::now();
    }

    // get some information from the header
    uint32_t _seq = msg->header.seq;
    ros::Time _stamp = msg->header.stamp;

    _logDebug("Start getting the image data");

    cv_bridge::CvImagePtr cv_ptr;
    if (_isDepthImage) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            if (monoImg == nullptr) {
                monoImg = new cv::Mat(cv_ptr->image.size(), CV_8UC1);
            }
            cv::convertScaleAbs(cv_ptr->image, *monoImg, 255.0 / _maxDepth, 0.0);
            cv::cvtColor(*monoImg, rgbImg, cv::COLOR_GRAY2RGB);
        } catch (const std::exception &e) {
            ROS_ERROR("DepthImage: cv_bridge exception: %s", e.what());
            std::cerr << e.what() << '\n';
            return;
        }
    } else if (_isGrayImage) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_ptr->image, rgbImg, cv::COLOR_GRAY2RGB);  // TODO: or maybe the pointer *cv_ptr->image
        } catch (const std::exception &e) {
            ROS_ERROR("Gray: cv_bridge exception: %s", e.what());
            std::cerr << e.what() << '\n';
            return;
        }
    } else {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            _logDebug("Successfully fetched image pointer");
        } catch (cv_bridge::Exception &e) {
            _logDebug("Error while getting the image pointer");
            ROS_ERROR("RGBImage: cv_bridge exception: %s", e.what());
            return;
        }
    }

    _logDebug("Now check for init");

    // do we have to initialize?
    if (_imageHeight == -1) {
        _logDebug("Yes we have to init");
        //assert(("Error during initialization", initialize(cv_ptr->image.cols, cv_ptr->image.rows, _decoderType)));
        _initialize(cv_ptr->image.cols, cv_ptr->image.rows, getTimeStampString(_stamp), _decoderType);
    }

    // write the image to the h264 file
    if (_isDepthImage || _isGrayImage) {
        videoWriter.write(rgbImg);
    } else {
        videoWriter.write(cv_ptr->image);
    }

    // write to the log file
    *logFile << _currentFrame << " " << _seq << " " << getTimeStampString(_stamp) << "\n";

    // scale the image and publish
    if (_publishScaledImage) {
        if (_isDepthImage || _isGrayImage) {
            cv::resize(rgbImg, scaledImg, cv::Size(), _imageScaleFactor, _imageScaleFactor);
        } else {
            cv::resize(cv_ptr->image, scaledImg, cv::Size(), _imageScaleFactor, _imageScaleFactor);
        }
        scaledImgBridge.header = msg->header;
        scaledImgBridge.image = scaledImg;
        scaledImgBridge.encoding = "bgr8";  // TODO: check if this is working for zFAS images
        scaledImgBridge.toImageMsg(scaledImgMsg);

        imagePub.publish(scaledImgMsg);
    }

    if (_currentFrame % 100 == 0) {
        ROS_INFO("%s H264 Encoder: saving frame %d", _debugPrefix.c_str(), _currentFrame);
    }

    if (debugmode) {
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        double _hz = 1.0 / elapsed.count();
        ROS_INFO("Capable of %f fps", _hz);
    }

    _currentFrame += 1;

    pub.publish(msg->header);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_h264_encoder");
    ros::NodeHandle n("~");

    // get some parameters
    n.getParam("fps", _fps);
    //n.getParam("logFilePath", logFilePath);
    n.getParam("decoderType", _decoderType);
    n.getParam("bitrate", _bitrate);
    n.getParam("targetLocation", _location);

    n.getParam("isDepthImage", _isDepthImage);
    n.getParam("isGrayImage", _isGrayImage);
    n.getParam("maxDepth", _maxDepth);

    n.getParam("publishScaledImage", _publishScaledImage);
    n.getParam("imageScaleFactor", _imageScaleFactor);

    n.getParam("debugPrefix", _debugPrefix);

    ros::Subscriber sub = n.subscribe("image", 1000, imageCallback);
    pub = n.advertise<std_msgs::Header>("header", 1000);

    if (_publishScaledImage) {
        imagePub = n.advertise<sensor_msgs::Image>("scaled_image", 1000);
    }

    ros::spin();

    if (logFile != nullptr) {
        logFile->close();
    }

    videoWriter.release();

    return 0;
}
