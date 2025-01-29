#ifndef DOOR_SERVER__DOOR_SERVER_
#define DOOR_SERVER__DOOR_SERVER_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>

class DoorServer : public rclcpp::Node
{
public:
    explicit DoorServer(const rclcpp::NodeOptions & options);

private:
    void infraImgSubCallback(const sensor_msgs::msg::Image::SharedPtr infra_img_msg);
    void depthImgSubCallback(const sensor_msgs::msg::Image::SharedPtr depth_img_msg);
    void doorServerTimerCallback();

    // Parameters
    int update_rate_{10};

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infra_img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
    rclcpp::TimerBase::SharedPtr door_server_timer_;

    // State variables
    cv_bridge::CvImagePtr infra_cv_img_ptr_;
    cv_bridge::CvImagePtr depth_cv_img_ptr_;
};

#endif  // DOOR_SERVER__DOOR_SERVER_