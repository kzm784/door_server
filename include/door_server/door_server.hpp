#ifndef DOOR_SERVER__DOOR_SERVER_
#define DOOR_SERVER__DOOR_SERVER_

#include <chrono>
#include <fstream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

class DoorServer : public rclcpp::Node
{
public:
    explicit DoorServer(const rclcpp::NodeOptions &options);

    void infraImgSubCallback(const sensor_msgs::msg::Image::SharedPtr infra_img_msg);
    void depthImgSubCallback(const sensor_msgs::msg::Image::SharedPtr depth_img_msg);
    void reachedWaypointSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    std::vector<int> loadDoorEventId(const std::string & file_name);
    void doorServerTimerCallback();

    void createDiffImage(
        const cv::Mat& src_img,
        const cv::Mat& src_img_prev,
        cv::Mat& result_img,
        const bool img_check);
    
    void labelingHoughTransform(
        const cv::Mat& src_img,
        cv::Mat& result_img,
        const bool img_check,
        const double vertical_angle,
        double strict_vertical_angle,
        int line_width);

    void kmeansHoughTransform(
        const cv::Mat src_img,
        cv::Mat& result_img,
        const bool img_check,
        const double vertical_angle,
        double strict_vertical_angle,
        cv::Mat& clustering_points);

    void kmeansClustering(
        const cv::Mat& clustering_points,
        int k,
        cv::Mat& result_img,
        const bool img_check);

    void updateDoorFrameData(
        const double ave_speed_threshold,
        const int interpolation_limit,
        const cv::Mat& src_img,
        cv::Mat& result_img,
        const bool img_check);

    void caluculateCenterFrameDistance(
        const std::vector<std::tuple<cv::Point2f, std::array<double, 10>, double,  int>>& door_frame_data_updated,
        std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos,
        cv::Mat& result_img,
        cv::Mat& depth_img,
        const bool img_check);

    void doorAvePosCalc(
        cv::Mat& src_img,
        std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos);

    void pixelToPoint(
        float x_pix,
        float y_pix,
        float depth,
        cv::Point3f& point);


private:
    // Parameters
    std::string event_id_path_;
    int update_rate_{10};
    bool check_diff_iamge_{false};
    bool check_labeling_hough_image_{false};
    bool check_kmeans_hough_image_{false};
    bool check_kmeans_clustering_image_{false};
    bool check_updated_image_{false};
    bool check_door_state_image_{false};

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infra_img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reached_waypoint_id_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr event_flag_pub_;
    rclcpp::TimerBase::SharedPtr door_server_timer_;

    // State variables
    cv_bridge::CvImagePtr infra_cv_img_ptr_;
    cv_bridge::CvImagePtr depth_cv_img_ptr_;
    cv::Mat img_prev_;
    
    //現在のドアフレームのデータ群 <0:x座標, 1:ラベル番号> の宣言
    std::vector<std::tuple<cv::Point2f, int>> door_frame_data_;
    
    //ドア開閉判定で使用するドアフレームのデータ群 <0:x座標, 1:過去の速度配列, 2:平均速度, 3:補間カウンター> の宣言
    std::vector<std::tuple<cv::Point2f, std::array<double, 10>, double,  int>> door_frame_data_updated_;

    //中心ドアフレームペアの画像座標と三次元座標を保存する変数の宣言
    std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>> door_pos_;
    
    //中心ドアフレームペアの補間に使用する 1 フレーム前のデータ群 <0: x座標, 1: 平均速度>
    std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>> door_pos_prev_;

    double min_diff_prev_ ;
    int state_;
    bool is_door_opened_;
    std::array<int, 10> door_state_factor_;

    std::vector<int> door_event_id_;
    bool activate_door_event_{false};
    int opened_state_{0};
};

#endif  // DOOR_SERVER__DOOR_SERVER_