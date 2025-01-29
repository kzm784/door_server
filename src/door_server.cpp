#include "door_server/door_server.hpp"

DoorServer::DoorServer(const rclcpp::NodeOptions & options)
: Node("door_server", options)
{
    // Parameters
    declare_parameter<std::int32_t>("update_rate", 10);
    get_parameter("update_rate", update_rate_);
    RCLCPP_INFO(get_logger(), "update_rate: %d", update_rate_);    

    // Subscribers
    infra_img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/infra1/image_rect_raw",
        rclcpp::QoS(10).best_effort().durability_volatile(),
        std::bind(&DoorServer::infraImgSubCallback, this, std::placeholders::_1)
    );
    depth_img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/depth/image_rect_raw",
        rclcpp::QoS(10).best_effort().durability_volatile(),
        std::bind(&DoorServer::depthImgSubCallback, this, std::placeholders::_1)
    );

    // Timer
    door_server_timer_ = create_wall_timer(
        std::chrono::milliseconds( 1000 / update_rate_),
        std::bind(&DoorServer::doorServerTimerCallback, this)    
    );
}

void DoorServer::infraImgSubCallback(const sensor_msgs::msg::Image::SharedPtr infra_img_msg)
{
    try
    {
        infra_cv_img_ptr_ = cv_bridge::toCvCopy(infra_img_msg, infra_img_msg->encoding);
        return;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exeption: %s", e.what());
        return;
    }
}

void DoorServer::depthImgSubCallback(const sensor_msgs::msg::Image::SharedPtr depth_img_msg)
{
    try
    {
        depth_cv_img_ptr_ = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        return;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exeption: %s", e.what());
        return;
    }
}

void DoorServer::doorServerTimerCallback()
{
    
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DoorServer)