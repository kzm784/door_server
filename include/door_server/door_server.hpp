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
    explicit DoorServer(const rclcpp::NodeOptions &options);

private:
    void infraImgSubCallback(const sensor_msgs::msg::Image::SharedPtr infra_img_msg);
    void depthImgSubCallback(const sensor_msgs::msg::Image::SharedPtr depth_img_msg);
    void doorServerTimerCallback();
    
    void labelingHoughTransform(
        const cv::Mat &src_img,
        const cv::Mat &result_img,
        double vertical_angle,
        double strict_vertical_angle,        
        int line_width);
    
    void kMeansHoughTransform(
        const cv::Mat &src_img,
        double vertical_angle,
        double strict_vertical_angle,
        cv::Mat &clustering_points);
    
    void kMeansClustering(
        const cv::Mat &clustering_points,
        int k);
    
    void updateDoorFrameData(
        double ave_speed_threshold,
        int interpolation_limit,
        const cv::Mat &src_img);

    void caluculateCenterFrameDistance(
        const std::vector<std::tuple<cv::Point2f, std::array<double, 10>, double,  int>>& door_frame_data_updated,
        std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos,
        const cv::Mat& result_img,
        cv::Mat& depth_image);

    void pixelToPoint(
        float x_pix,
        float y_pix,
        float depth,
        cv::Point3f& point);

    void doorPixelToPoint(
        cv::Mat& src_img,
        std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos);

    void doorAvePosCalc(
        cv::Mat& src_img,
        std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos);


    
    // Parameters
    int update_rate_{10};

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infra_img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
    rclcpp::TimerBase::SharedPtr door_server_timer_;

    // State variables
    cv_bridge::CvImagePtr infra_cv_img_ptr_;
    cv_bridge::CvImagePtr depth_cv_img_ptr_;
    cv::Mat img_prev_;
    std::vector<std::tuple<cv::Point2f, int>> door_frame_data_;
    std::vector<std::tuple<cv::Point2f, std::array<double, 10>, double, int>> door_frame_data_updated_;
    std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>> door_pos_;
    std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>> door_pos_prev_;
    double min_diff_prev_{0.0};
    int state_{0};
    bool is_door_opened_{false};
    std::array<int, 10> door_state_factor_;
};

#endif  // DOOR_SERVER__DOOR_SERVER_