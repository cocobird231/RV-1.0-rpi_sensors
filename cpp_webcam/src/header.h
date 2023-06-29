#include <vector>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg/image.hpp"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"

#include <opencv2/opencv.hpp>

#define TS_MODE

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string topic_Webcam_nodeName = "webcam_publisher_node";
    std::string topic_Webcam_topicName = "topic_Webcam";
    float topic_Webcam_pubInterval_s = 0.03;
    int topic_Webcam_width = 1920;
    int topic_Webcam_height = 1080;
    int camera_cap_id = 0;
    float camera_fps = 30.0;
    int camera_width = 1920;
    int camera_height = 1080;
    bool camera_use_color = true;

private:
    void _getParams()
    {
        this->get_parameter("topic_Webcam_nodeName", this->topic_Webcam_nodeName);
        this->get_parameter("topic_Webcam_topicName", this->topic_Webcam_topicName);
        this->get_parameter("topic_Webcam_pubInterval_s", this->topic_Webcam_pubInterval_s);
        this->get_parameter("topic_Webcam_width", this->topic_Webcam_width);
        this->get_parameter("topic_Webcam_height", this->topic_Webcam_height);
        this->get_parameter("camera_cap_id", this->camera_cap_id);
        this->get_parameter("camera_fps", this->camera_fps);
        this->get_parameter("camera_width", this->camera_width);
        this->get_parameter("camera_height", this->camera_height);
        this->get_parameter("camera_use_color", this->camera_use_color);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("topic_Webcam_nodeName", this->topic_Webcam_nodeName);
        this->declare_parameter<std::string>("topic_Webcam_topicName", this->topic_Webcam_topicName);
        this->declare_parameter<float>("topic_Webcam_pubInterval_s", this->topic_Webcam_pubInterval_s);
        this->declare_parameter<int>("topic_Webcam_width", this->topic_Webcam_width);
        this->declare_parameter<int>("topic_Webcam_height", this->topic_Webcam_height);
        this->declare_parameter<int>("camera_cap_id", this->camera_cap_id);
        this->declare_parameter<float>("camera_fps", this->camera_fps);
        this->declare_parameter<int>("camera_width", this->camera_width);
        this->declare_parameter<int>("camera_height", this->camera_height);
        this->declare_parameter<bool>("camera_use_color", this->camera_use_color);
        this->_getParams();
    }
};

#ifdef TS_MODE
class RGBImagePublisher : public TimeSyncNode
#else
class RGBImagePublisher : public rclcpp::Node
#endif
{
private:
    std::shared_ptr<Params> params;
    rclcpp::Publisher<vehicle_interfaces::msg::Image>::SharedPtr pub_;
    std::string nodeName_;

public:
    RGBImagePublisher(const std::shared_ptr<Params>& params) : 
#ifdef TS_MODE
        TimeSyncNode(params->nodeName, params->timesyncService, 100000, 2), 
#endif
        rclcpp::Node(params->nodeName), 
        params(params)
    {
        this->nodeName_ = params->nodeName;
        this->pub_ = this->create_publisher<vehicle_interfaces::msg::Image>(params->topic_Webcam_topicName, 10);
    }

    void pubImage(const std::vector<uchar>& dataVec, const cv::Size& sz)
    {
        static u_int64_t frame_id = 0;
        auto msg = vehicle_interfaces::msg::Image();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_SENSOR;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_IMAGE;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = frame_id++;
#ifdef TS_MODE
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration();
#else
        msg.header.stamp_type = vehicle_interfaces::msg::Header::STAMPTYPE_NONE_UTC_SYNC;
        msg.header.stamp = this->get_clock()->now();
        msg.header.stamp_offset = rclcpp::Duration(0, 0);
#endif
        msg.header.ref_publish_time_ms = this->params->topic_Webcam_pubInterval_s * 1000.0;

        msg.format_type = msg.FORMAT_JPEG;
        msg.width = sz.width;
        msg.height = sz.height;
        msg.data = dataVec;
        this->pub_->publish(msg);
    }
};

#ifdef TS_MODE
class RGBImageSubscriber : public TimeSyncNode
#else
class RGBImageSubscriber : public rclcpp::Node
#endif
{
private:
    rclcpp::Subscription<vehicle_interfaces::msg::Image>::SharedPtr subscription_;
    std::string nodeName_;

    cv::Mat recvMat_;
    std::atomic<bool> initMatF_;
    std::atomic<bool> newMatF_;
    std::mutex recvMatLock_;

private:
    void _topic_callback(const vehicle_interfaces::msg::Image::SharedPtr msg)
    {
        std::vector<uchar> data = msg->data;
        std::unique_lock<std::mutex> lockMat(this->recvMatLock_, std::defer_lock);
        lockMat.lock();
        try
        {
            this->recvMat_ = cv::imdecode(data, 1);
            this->newMatF_ = true;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        catch(...)
        {
            std::cerr << "Unknown Exception.\n";
        }
        lockMat.unlock();
    }

public:
    RGBImageSubscriber(const std::shared_ptr<Params>& params, const cv::Mat& initMat) : 
#ifdef TS_MODE
        TimeSyncNode(params->nodeName, params->timesyncService, 100000, 2), 
#endif
        rclcpp::Node(params->nodeName)
    {
        this->setInitMat(initMat);
        this->nodeName_ = params->nodeName;
        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::Image>(params->topic_Webcam_topicName, 
            10, std::bind(&RGBImageSubscriber::_topic_callback, this, std::placeholders::_1));
    }

    void setInitMat(const cv::Mat& img)
    {
        std::unique_lock<std::mutex> lockMat(this->recvMatLock_, std::defer_lock);
        lockMat.lock();
        this->recvMat_ = img;
        this->initMatF_ = true;
        lockMat.unlock();
    }

    bool getRecvMat_clone(cv::Mat& out)
    {
        bool ret = false;
        if (this->initMatF_ && this->newMatF_)
        {
            std::unique_lock<std::mutex> lockMat(this->recvMatLock_, std::defer_lock);
            lockMat.lock();
            out = this->recvMat_.clone();
            this->newMatF_ = false;
            lockMat.unlock();
            ret = true;
        }
        return ret;
    }
};

class WorkingRate
{
private:
	float rate_;
	int frameCnt_;
	std::chrono::duration<int, std::milli> interval_ms_;
	Timer* timer_;

	std::mutex locker_;

private:
	template <typename T>
	void _safeSave(T* ptr, const T value, std::mutex& lock)
	{
		std::lock_guard<std::mutex> _lock(lock);
		*ptr = value;
	}

	template <typename T>
	T _safeCall(const T* ptr, std::mutex& lock)
	{
		std::lock_guard<std::mutex> _lock(lock);
		return *ptr;
	}

	void _timerCallback()
	{
		int cnt = this->_safeCall(&this->frameCnt_, this->locker_);
		std::chrono::duration<float> toSec = this->interval_ms_;// Casting msec to sec
		this->_safeSave(&this->rate_, (float)cnt / toSec.count(), this->locker_);
		this->_safeSave(&this->frameCnt_, 0, this->locker_);
	}

public:
	WorkingRate(int interval_ms)
	{
		this->rate_ = 0;
		this->frameCnt_ = 0;
		this->interval_ms_ = std::chrono::milliseconds(interval_ms);
		this->timer_ = new Timer(interval_ms, std::bind(&WorkingRate::_timerCallback, this));
	}

	~WorkingRate()
	{
		this->timer_->destroy();
		delete this->timer_;
	}

	void addCnt(int num) { this->_safeSave(&this->frameCnt_, this->frameCnt_ + num, this->locker_); }

	void addOneCnt() { this->_safeSave(&this->frameCnt_, this->frameCnt_ + 1, this->locker_); }

	void start() { this->timer_->start(); }

	void stop() { this->timer_->stop(); }

	float getRate() { return this->_safeCall(&this->rate_, this->locker_); }
};

void SpinNode(std::shared_ptr<rclcpp::Node> node, std::string threadName)
{
	std::cerr << threadName << " start..." << std::endl;
	rclcpp::spin(node);
	std::cerr << threadName << " exit." << std::endl;
	rclcpp::shutdown();
}