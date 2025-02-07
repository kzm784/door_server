#include "door_server/door_server.hpp"

DoorServer::DoorServer(const rclcpp::NodeOptions & options)
: Node("door_server", options)
{
    // Parameters
    declare_parameter<std::string>("event_id_path", "");
    declare_parameter<std::int32_t>("update_rate", 10);
    declare_parameter<bool>("check_diff_image", false);
    declare_parameter<bool>("check_labeling_hough_image", false);
    declare_parameter<bool>("check_kmeans_hough_image", false);
    declare_parameter<bool>("check_kmeans_clustering_image", false);
    declare_parameter<bool>("check_updated_image", false);
    declare_parameter<bool>("check_door_state_image", false);

    get_parameter("event_id_path", event_id_path_);
    get_parameter("update_rate", update_rate_);
    get_parameter("check_diff_image", check_diff_iamge_);
    get_parameter("check_labeling_image", check_labeling_hough_image_);
    get_parameter("check_kmeans_hough_image", check_kmeans_hough_image_);
    get_parameter("check_kmeans_clustering_image", check_kmeans_clustering_image_);
    get_parameter("check_updated_image", check_updated_image_);
    get_parameter("check_door_state_image", check_door_state_image_);

    RCLCPP_INFO(get_logger(), "evnet_id_path:%s", event_id_path_.c_str());
    RCLCPP_INFO(get_logger(), "update_rate: %d", update_rate_);
    RCLCPP_INFO(get_logger(), "check_diff_image: %s", check_diff_iamge_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "check_labeling_hough_image: %s", check_labeling_hough_image_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "check_kmeans_hough_image: %s", check_kmeans_hough_image_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "check_kmeans_clustering_image: %s", check_kmeans_clustering_image_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "check_updated_image: %s", check_updated_image_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "check_door_state_image: %s", check_door_state_image_ ? "true" : "false");

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

    reached_waypoint_id_sub_ = create_subscription<std_msgs::msg::Int32>(
      "reached_waypoint_id",
      rclcpp::QoS(10),
      std::bind(&DoorServer::reachedWaypointSubCallback, this, std::placeholders::_1)
    );

    event_flag_pub_ = this->create_publisher<std_msgs::msg::Int32>("event_flag", rclcpp::QoS(10));


    // Timer
    door_server_timer_ = create_wall_timer(
        std::chrono::milliseconds( 1000 / update_rate_),
        std::bind(&DoorServer::doorServerTimerCallback, this)
    );

    door_event_id_ = loadDoorEventId(event_id_path_);
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

void DoorServer::reachedWaypointSubCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int reached_id = msg->data;
    if (std::find(door_event_id_.begin(), door_event_id_.end(), reached_id) != door_event_id_.end())
    {
        activate_door_event_ = true;
        RCLCPP_INFO(get_logger(), "activate_door_event set to true, reached waypoint id: %d", reached_id);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Reached waypoint id %d is not in door_event_id_", reached_id);
    }
}

std::vector<int> DoorServer::loadDoorEventId(const std::string &file_name)
{
    std::ifstream file(file_name);
    if (!file.is_open())
    {
        RCLCPP_INFO(rclcpp::get_logger("door_server"), "No IDs loaded");
        return {};
    }

    std::vector<int> event_id;
    std::string line;
    
    // ヘッダー行を読み飛ばす（1行目をスキップ）
    if (!std::getline(file, line)) {
        RCLCPP_WARN(rclcpp::get_logger("door_server"), "Failed to read header from file: %s", file_name.c_str());
        return event_id;
    }
    
    // 2行目以降を読み込み
    while (std::getline(file, line))
    {
        if (line.empty())
            continue;
            
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> row;
        
        // カンマで分割（1列の場合でも、1要素の vector となる）
        while (std::getline(ss, token, ','))
        {
            row.push_back(token);
        }
        
        // 少なくとも1列はあるか確認
        if (row.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("door_server"), "Empty row encountered: %s", line.c_str());
            continue;
        }
        
        try {
            // 1列目（row[0]）の値を整数に変換して push_back する
            event_id.push_back(std::stoi(row[0]));
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("door_server"), "Conversion error for line: %s", line.c_str());
            continue;
        }
    }
    
    return event_id;
}


void DoorServer::doorServerTimerCallback()
{
  if (!activate_door_event_) {
      return;
  }

  if (infra_cv_img_ptr_ && depth_cv_img_ptr_)
  {
    if (!img_prev_.empty())
    {
      door_frame_data_.clear();

      cv::Mat verifical_color_img = cv::Mat::zeros(infra_cv_img_ptr_->image.size(), CV_8UC4);

      // 差分画像の作成
      cv::Mat diff_img = cv::Mat::zeros(infra_cv_img_ptr_->image.size(), CV_8UC1);
      createDiffImage(infra_cv_img_ptr_->image, img_prev_, diff_img, check_diff_iamge_);
      // cv::absdiff(infra_cv_img_ptr_->image, img_prev_, diff_img);

      // 得られたdiff_imageに対してハフ変換を行いラベリングを行う
      cv::Mat labeling_img = cv::Mat::zeros(infra_cv_img_ptr_->image.size(), CV_8UC1);
      labelingHoughTransform(diff_img, labeling_img, check_labeling_hough_image_, 3.0, 0.5, 5);
      int nlabel = cv::connectedComponents(labeling_img, labeling_img, 8);
      // RCLCPP_INFO(this->get_logger(), "label: %d", nlabel-1);

      //diff_imageをハフ変換し、その座標をclustering_pointsに代入
      cv::Mat clustering_points;   //クラスタリング材料
      kmeansHoughTransform(diff_img, verifical_color_img, check_kmeans_hough_image_, 3.0, 0.5, clustering_points);
      
      //clustering_pointsをラベリングで得られた数でクラスタリングし、ラベル、座標を並び替え、その後、それらのラベル、座標を door_frame_data_ に代入（速度はゼロで初期化）
      kmeansClustering(clustering_points, nlabel-1, verifical_color_img, check_kmeans_clustering_image_);
      // RCLCPP_INFO(this->get_logger(), "door_frame_data size: %d", (int)door_frame_data_.size());

      //瞬間速度、平均速度を door_frame_data_updated_に代入
      updateDoorFrameData(5.0, 2, verifical_color_img, verifical_color_img, check_updated_image_);
      // RCLCPP_INFO(this->get_logger(), "now size: %d, updated size : %d", (int)door_frame_data_.size(), (int)door_frame_data_updated_.size());

      caluculateCenterFrameDistance(door_frame_data_updated_, door_pos_, verifical_color_img, depth_cv_img_ptr_->image, check_door_state_image_);

      infra_cv_img_ptr_->image.copyTo(img_prev_);
      cv::waitKey(1);
    }
    else
    {
      try
      {
        infra_cv_img_ptr_->image.copyTo(img_prev_);
        return;
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "copyTo exeption: %s", e.what());
        return; 
      }
        
    }
  }
}

void DoorServer::createDiffImage(
  const cv::Mat& src_img,
  const cv::Mat& src_img_prev,
  cv::Mat& result_img,
  const bool img_check)
  {
    cv::absdiff(src_img, src_img_prev, result_img);
    cv::threshold(result_img, result_img, 20, 255, cv::THRESH_BINARY);
    
    if (img_check == true)
    {
      cv::imshow("infra_image", src_img);
      cv::imshow("image_prev", src_img_prev);
      cv::imshow("diff_image", result_img);
    }
  }

void DoorServer::labelingHoughTransform(
  const cv::Mat& src_img,
  cv::Mat& result_img,
  const bool img_check,
  const double vertical_angle,
  double strict_vertical_angle,
  int line_width)   
{
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(src_img, lines, 1, CV_PI / 180, 50);

  for (const auto& line : lines)
  {
    double rho = line[0];
    double theta = line[1];

    if (fabs(theta - CV_PI) <= vertical_angle * CV_PI / 180.0 || fabs(theta) <= vertical_angle * CV_PI / 180.0)
    {
      cv::Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a * rho, y0 = b * rho;

      if (fabs(theta - CV_PI) <= strict_vertical_angle * CV_PI / 180.0 || fabs(theta) <= strict_vertical_angle * CV_PI / 180.0)
      {
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));

        if (pt1.x < 0) pt1.x = 0;
        if (pt2.x < 0) pt2.x = 0;
        if (pt1.x >= src_img.cols) pt1.x = src_img.cols -1;
        if (pt2.x >= src_img.cols) pt2.x = src_img.cols -1;

        if (pt1.y < 0) pt1.y = 0;
        if (pt2.y < 0) pt2.y = 0;
        if (pt1.y >= src_img.rows) pt1.y = src_img.rows - 1;
        if (pt2.y >= src_img.rows) pt2.y = src_img.rows - 1;

        cv::line(result_img, pt1, pt2, cv::Scalar(255, 0, 0), line_width);
      }
      else
      {
        double a = -1.0 / tan(theta);
        double b = rho / sin(theta);
        pt1.x = (-b) / a;
        pt1.y = 0;
        pt2.x = (src_img.rows-1 - b) / a;
        pt2.y = src_img.rows-1;

        cv::line(result_img, pt1, pt2, cv::Scalar(255, 0, 0), line_width);
      }
    }
  }

  if (img_check == true)
  {
    cv::imshow("result: labeling_hough_transform_", result_img);
  }
}

void DoorServer::kmeansHoughTransform(
  const cv::Mat src_img,
  cv::Mat& result_img,
  const bool img_check,
  const double vertical_angle,
  double strict_vertical_angle,
  cv::Mat& clustering_points)
{
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(src_img, lines, 1, CV_PI / 180, 50);

  std::vector<cv::Point> pt1(lines.size()), pt2(lines.size());

  int line_num = 0;

  for (const auto& line : lines)
  {
    double rho = line[0];
    double theta = line[1];
    
    if (fabs(theta - CV_PI) <= vertical_angle * CV_PI / 180.0 || fabs(theta) <= vertical_angle * CV_PI / 180.0)
    {
      if (fabs(theta - CV_PI) <= strict_vertical_angle * CV_PI / 180.0 || fabs(theta) <= strict_vertical_angle * CV_PI / 180.0)
      {
        double at = cos(theta), bt = sin(theta);
        double x0 = at * rho, y0 = bt * rho;
        pt1[line_num].x = cvRound(x0 + 1000 * (-bt));
        pt1[line_num].y = cvRound(y0 + 1000 * (at));
        pt2[line_num].x = cvRound(x0 - 1000 * (-bt));
        pt2[line_num].y = cvRound(y0 - 1000 * (at));

        if (pt1[line_num].x < 0) pt1[line_num].x = 0;
        if (pt2[line_num].x < 0) pt2[line_num].x = 0;
        if (pt1[line_num].x >= src_img.cols) pt1[line_num].x = src_img.cols -1;
        if (pt2[line_num].x >= src_img.cols) pt2[line_num].x = src_img.cols -1;

        if (pt1[line_num].y < 0) pt1[line_num].y = 0;
        if (pt2[line_num].y < 0) pt2[line_num].y = 0;
        if (pt1[line_num].y >= src_img.rows) pt1[line_num].y = src_img.rows - 1;
        if (pt2[line_num].y >= src_img.rows) pt2[line_num].y = src_img.rows - 1;
      }
      else
      {
        double a = -1.0 / tan(theta);
        double b = rho / sin(theta);
        pt1[line_num].x = (-b) / a;
        pt1[line_num].y = 0;
        pt2[line_num].x = (src_img.rows-1 - b) / a;
        pt2[line_num].y = src_img.rows-1;
      }
      line_num++;
    }
  }
  
  //引数のclustering_pointsはここで初期化を行う
  clustering_points = cv::Mat(line_num, 2, CV_32F);
  for (int i = 0; i < line_num; i++)
  {
    clustering_points.at<float>(i, 0) = (pt1[i].x + pt2[i].x) / 2.0;
    clustering_points.at<float>(i, 1) = (pt1[i].x + pt2[i].x) / 2.0;

    //result_imgに描画
    if (img_check == true) cv::line(result_img, cv::Point(clustering_points.at<float>(i, 0), 0), cv::Point(clustering_points.at<float>(i, 0), src_img.cols-1), cv::Scalar(255,255,255), 1);
  }

  //確認用
  if (img_check == true)
  {
    cv::imshow("result: k_means_hough_transform_", result_img);
  }
}   

void DoorServer::kmeansClustering(
  const cv::Mat& clustering_points,
  int k,
  cv::Mat& result_img,
  const bool img_check)
{
  //確認用
  cv::Scalar colorTab[]=
  {
    cv::Scalar(0, 255, 255),
    cv::Scalar(255,255,0),
    cv::Scalar(0,255,0),
    cv::Scalar(0,0,255),

    cv::Scalar(255,255,0),
    cv::Scalar(100, 255, 255),
    cv::Scalar(255,100,100),
    cv::Scalar(100,255,100),

    cv::Scalar(100,100,255),
    cv::Scalar(255,255,100),    
    cv::Scalar(255,100,255), 
    cv::Scalar(255,0,255),
    cv::Scalar(255,255,255)
  };

  if (img_check == true)
  {
    for (size_t i = 0; i < door_frame_data_.size(); i++)
    {
      cv::line(result_img, cv::Point(std::get<0>(door_frame_data_[i]).x, 0), cv::Point(std::get<0>(door_frame_data_[i]).x, result_img.cols-1), colorTab[std::get<1>(door_frame_data_[i])], 3);
    }
    cv::imshow("result: k_means_clustering", result_img);
  }

  if (k < 1) return;

  //k-meansでクラスタリング
  cv::Mat labels, centers;
  cv::kmeans(clustering_points, k, labels, cv::TermCriteria(cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0)), 3, cv::KMEANS_PP_CENTERS, centers);

  //クラスタリングしたdoor_frameの座標、ラベルをdoor_frame_dataに代入、瞬間速度は0.0で初期化
  int door_frame_data_size = 0;
  for (int i = 0; i < k; i++)
  {
    door_frame_data_.push_back(std::make_tuple(cv::Point2f(centers.at<float>(i, 0), centers.at<float>(i, 1)), i));
    door_frame_data_size++;
  }

  //ドアの座標を小さい順に並び替え、その後小さい順にラベルを振り分ける
  std::sort(door_frame_data_.begin(), door_frame_data_.end(), [](const std::tuple<cv::Point2f, int>& a, const std::tuple<cv::Point2f, int>& b) {return std::get<0>(a).x < std::get<0>(b).x; });
  for (int i = 0; i < labels.rows; i++) 
  {
    for (int j = 0; j < k; j++) 
    {
      if (labels.at<int>(i) == std::get<1>(door_frame_data_[j]))
      {
        labels.at<int>(i) = j;
        break;
      }
    }
  }
  for (int j = 0; j < k; j++) {
    std::get<1>(door_frame_data_[j]) = j;
  }

  //確認用
  if (img_check == true)
  {
    for (size_t i = 0; i < door_frame_data_.size(); i++)
    {
      cv::line(result_img, cv::Point(std::get<0>(door_frame_data_[i]).x, 0), cv::Point(std::get<0>(door_frame_data_[i]).x, result_img.cols-1), colorTab[std::get<1>(door_frame_data_[i])], 3);
    }
    cv::imshow("result: k_means_clustering", result_img);
    result_img = cv::Mat::zeros(result_img.size(), CV_8UC4);
  }
}        

void DoorServer::updateDoorFrameData(
  const double ave_speed_threshold,
  const int interpolation_limit,
  const cv::Mat& src_img,
  cv::Mat& result_img,
  const bool img_check)
{
  // door_frame_data_updated_ の初期化  (動作確認済み)
  if (door_frame_data_.empty() && door_frame_data_updated_.empty())
  {
    door_frame_data_updated_.clear();
    return;
  }
  else if (door_frame_data_updated_.empty() && !door_frame_data_.empty())
  {
    door_frame_data_updated_.clear();
    for (size_t i = 0; i < door_frame_data_.size(); i++)
    {
      door_frame_data_updated_.push_back(std::make_tuple(cv::Point(std::get<0>(door_frame_data_[i])), std::array<double, 10>{0.0}, 0.0, 0));
    }
  }

  // x 座標が小さい順に並び替える (動作確認済み)
  std::sort
  (
    door_frame_data_updated_.begin(), door_frame_data_updated_.end(),
    [](const std::tuple<cv::Point2f, std::array<double, 10>, double, int>& a, const std::tuple<cv::Point2f, std::array<double, 10>, double, int>& b)
    {
      return std::get<0>(a).x < std::get<0>(b).x;
    }
  );
  // for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
  // {
  //   RCLCPP_INFO(this->get_logger(), "i: %d, x: %f", i, std::get<0>(door_frame_data_updated_[i]).x);
  // }


  // 瞬間速度の過去化登録   (動作確認済み)
  for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
  {
    // RCLCPP_INFO(this->get_logger(), "i: %d, sp: %f", i, std::get<2>(door_frame_data_updated_[i])[0]);
    for (size_t j = std::get<1>(door_frame_data_updated_[i]).size() - 1; j > 0; j--)
    {
      //過去フレームの瞬間速度の過去化登録
      std::get<1>(door_frame_data_updated_[i])[j] = std::get<1>(door_frame_data_updated_[i])[j-1];
    }
    //最新のフレームの瞬間速度は 0.0 で初期化する
    std::get<1>(door_frame_data_updated_[i])[0] = 0.0;
  }
  

  ///////////////////////////////////////////////////////////////// x 座標と瞬間速度の登録//////////////////////////////////////////////////////////////////////

  //最新の情報が欠損していた場合
  if (door_frame_data_.empty() && !door_frame_data_updated_.empty())
  {
    for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
    {
      double x_updated = std::get<0>(door_frame_data_updated_[i]).x;
      double sp_ave = std::get<2>(door_frame_data_updated_[i]);

      // x 座標の補間
      std::get<0>(door_frame_data_updated_[i]).x = x_updated + sp_ave;
      //瞬間速度の補間
      std::get<1>(door_frame_data_updated_[i])[0] = sp_ave;
      // std::get<1>(door_frame_data_updated_[i])[0] = 0.0;
      //補間カウンターを +1 する
      std::get<3>(door_frame_data_updated_[i]) += 1;
    }
    // RCLCPP_INFO(this->get_logger(), "door_frame_data_.empty() && !door_frame_data_updated_.empty()");
  }

  // door_frame_data_updated_ の数が door_frame_data_ 以上であった場合
  else if(door_frame_data_updated_.size() >= door_frame_data_.size())
  {
    for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
    {
      int min_j = 10;
      double sp_instant = 0.0;
      double min_diff = src_img.cols - 1;

      for (size_t j = 0; j < door_frame_data_.size(); j++)
      {
        double x_updated = std::get<0>(door_frame_data_updated_[i]).x;
        double x_now = std::get<0>(door_frame_data_[j]).x;
        double diff = x_now - x_updated;

        if (fabs(diff) < fabs(min_diff))
        {
          min_j = j;
          min_diff = diff;
          sp_instant = diff;
        }
      }

      double x_updated = std::get<0>(door_frame_data_updated_[i]).x;
      double sp_ave = std::get<2>(door_frame_data_updated_[i]);
      double x_now = std::get<0>(door_frame_data_[min_j]).x;
      
      if (x_updated - ave_speed_threshold <= x_now && x_now <= x_updated + ave_speed_threshold)
      {
        std::get<0>(door_frame_data_updated_[i]).x = x_now;
        std::get<1>(door_frame_data_updated_[i])[0] = sp_instant;
        std::get<3>(door_frame_data_updated_[i]) = 0;
      }
      else
      {
        //過去の x 座標とこれまでの平均速度で最新の x 座標を補間する
        std::get<0>(door_frame_data_updated_[i]).x = x_updated + sp_ave;
        //瞬間速度は過去の平均速度で上書きする
        std::get<1>(door_frame_data_updated_[i])[0] = sp_ave;
        // std::get<1>(door_frame_data_updated_[i])[0] = 0.0;
        //補間カウンターを +1 する
        std::get<3>(door_frame_data_updated_[i]) += 1;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "door_frame_data_updated_.size() >= door_frame_data_.size()");
  }

  //最新のフレームで door_frame_data_ の数が増えたとき
  else if (door_frame_data_updated_.size() < door_frame_data_.size())
  {
    //要素追加で使用する
    std::vector<int> update_num;

    for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
    {
      int min_j = 10;
      double sp_instant = 0.0;
      double min_diff = src_img.cols - 1;

      for (size_t j = 0; j < door_frame_data_.size(); j++)
      {
        double x_updated = std::get<0>(door_frame_data_updated_[i]).x;
        double x_now = std::get<0>(door_frame_data_[j]).x;
        double diff = x_now - x_updated;

        if (fabs(diff) < fabs(min_diff))
        {
          min_j = j;
          min_diff = diff;
          sp_instant = diff;
        }
      }

      //引き継いだ door_frame_data_ の要素番号を記憶
      update_num.push_back((int)min_j);

      double x_updated = std::get<0>(door_frame_data_updated_[i]).x;
      double sp_ave = std::get<2>(door_frame_data_updated_[i]);
      double x_now = std::get<0>(door_frame_data_[min_j]).x;
      
      if (x_updated - ave_speed_threshold <= x_now && x_now <= x_updated + ave_speed_threshold)
      {
        std::get<0>(door_frame_data_updated_[i]).x = x_now;
        std::get<1>(door_frame_data_updated_[i])[0] = sp_instant;
        std::get<3>(door_frame_data_updated_[i]) = 0;
      }
      else
      {
        //過去の x 座標とこれまでの平均速度で最新の x 座標を補間する
        std::get<0>(door_frame_data_updated_[i]).x = x_updated + sp_ave;
        //瞬間速度は過去の平均速度で上書きする
        std::get<1>(door_frame_data_updated_[i])[0] = sp_ave;
        // std::get<1>(door_frame_data_updated_[i])[0] = 0.0;
        //補間カウンターを +1 する
        std::get<3>(door_frame_data_updated_[i]) += 1;
      }
    }

    for (size_t i = 0; i < door_frame_data_.size(); i++)
    {
      bool skip = false;

      for (size_t j = 0; j < update_num.size(); j++)
      {
        if (i == static_cast<size_t>(update_num[j]))
        {
          skip = true;
          break;
        }
      }

      // if (!skip && i >= initial_size)
      if (!skip)
      {
        door_frame_data_updated_.push_back(std::make_tuple(cv::Point(std::get<0>(door_frame_data_[i])), std::array<double, 10>{0.0}, 0.0, 0));
      }
    }
    // RCLCPP_INFO(this->get_logger(), "door_frame_data_updated_.size() < door_frame_data_.size()");
  }

  //それ以外の場合があった場合の確認
  // else
  // {
  //   RCLCPP_WARN(this->get_logger(), "Something wrong!: prev: %d, now: %d", (int)door_frame_data_updated_.size(), (int)door_frame_data_.size());
  // }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // door_frame_data_updated_ における、これまでの速度情報から平均速度を計算
  for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
  {
    int div_counter = 0;
    double sp_sum = 0.0;
    double sp_ave = 0.0;
    for (size_t j = 0; j < std::get<1>(door_frame_data_updated_[i]).size(); j++)
    {
      sp_sum += std::get<1>(door_frame_data_updated_[i])[j];
      if (std::get<1>(door_frame_data_updated_[i])[j] != 0.0)
      {
        div_counter++;
      }
    }
    if (div_counter != 0)
    {
      sp_ave = sp_sum / div_counter;
    }
    else
    {
      sp_ave = 0.0;
    }
    // 平均速度の登録
    std::get<2>(door_frame_data_updated_[i]) = sp_ave;
    // RCLCPP_INFO(this->get_logger(), "sp_ave[%d]: %f", i, sp_ave);
  }


  // interpolation_limit 回補間していたデータの消去
  door_frame_data_updated_.erase
  (
    std::remove_if
    (
      door_frame_data_updated_.begin(), door_frame_data_updated_.end(), 
      [interpolation_limit](const std::tuple<cv::Point2f, std::array<double, 10>, double, int>& tuple)
      {
        return std::get<3>(tuple) > interpolation_limit;
      }
    ), door_frame_data_updated_.end()
  );


  // 確認用
  if (img_check == true)
  {
    for (size_t i = 0; i < door_frame_data_updated_.size(); i++)
    {
      if(std::get<2>(door_frame_data_updated_[i]) > 0.0)
      {
        cv::line(result_img, cv::Point(std::get<0>(door_frame_data_updated_[i]).x, 0), cv::Point(std::get<0>(door_frame_data_updated_[i]).x, result_img.rows -1), cv::Scalar(0,0,255), 3);
      }
      else if (std::get<2>(door_frame_data_updated_[i]) < -0.0)
      {
        cv::line(result_img, cv::Point(std::get<0>(door_frame_data_updated_[i]).x, 0), cv::Point(std::get<0>(door_frame_data_updated_[i]).x, result_img.rows -1), cv::Scalar(255,255,0), 3);
      }
      else
      {
        cv::line(result_img, cv::Point(std::get<0>(door_frame_data_updated_[i]).x, 0), cv::Point(std::get<0>(door_frame_data_updated_[i]).x, result_img.rows -1), cv::Scalar(0,255,0), 3);
      }
    }
    cv::imshow("result: update_door_frame_data_", result_img);
  }
}  

void DoorServer::caluculateCenterFrameDistance(
  const std::vector<std::tuple<cv::Point2f, std::array<double, 10>, double,  int>>& door_frame_data_updated,
  std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos,
  cv::Mat& result_img,
  cv::Mat& depth_img,
  const bool img_check)
{
  // door_state_factor_ の過去化
  for (size_t i = door_state_factor_.size() - 1; i > 0; i--)
  {
    door_state_factor_[i] = door_state_factor_[i-1];
  }
  door_state_factor_[0] = 0.0;

  
  int min_i = -1;
  int min_j = -1;
  if (door_frame_data_.size() >= 2)
  {
    //平均速度の積が負になるペアのうち、x 座標の差が最小になるペアを見つける
    double min_diff = result_img.cols - 1;
    for (size_t i = 0; i < door_frame_data_updated.size(); i++)
    {
      for (size_t j = 0; j < door_frame_data_updated.size(); j++)
      {
        if (i == j) break;
        double u_i = std::get<double>(door_frame_data_updated[i]);
        double u_j = std::get<double>(door_frame_data_updated[j]);
        double multiplied_value = u_i * u_j;
        double diff = result_img.cols - 1;
        if (multiplied_value < 0.0)
        {
          diff = fabs(std::get<cv::Point2f>(door_frame_data_updated[i]).x - std::get<cv::Point2f>(door_frame_data_updated[j]).x);
          if (diff < min_diff)
          {
            min_diff = diff;
            min_i = i;
            min_j = j;
          }
        }
      }
    }

    //min_diff_prev_ との差を取り、door_state_factor_[0] に現在の状態を入力する。(開：1, 閉：-1)
    double diff = min_diff - min_diff_prev_;
    if(diff > 0.0)
    {
      door_state_factor_[0] = 1;           //開いているとき
      // RCLCPP_INFO(this->get_logger(), "opening");
    } 
    else if (diff < 0.0)
    {
      door_state_factor_[0] = -1;    //閉じているとき
      // RCLCPP_WARN(this->get_logger(), "closing");
    } 
    else
    {
      door_state_factor_[0] = 0;                   //差がゼロ->閉じきったもしくは開ききった時
      // RCLCPP_ERROR(this->get_logger(), "unknown");
    } 

    //state_が 1 もしくは -1 つまり開閉どちらかの途中には計測する
    if (state_ != 0)
    {
      door_pos_prev_.first.first.x = std::get<0>(door_frame_data_updated[min_i]).x;
      door_pos_prev_.first.first.y = result_img.rows / 2;
      door_pos_prev_.second.first.x = std::get<0>(door_frame_data_updated[min_j]).x;
      door_pos_prev_.second.first.y = result_img.rows / 2;
    }
    // RCLCPP_INFO(this->get_logger(), "size: %d, x1: %f, x2: %f", door_frame_data_updated.size(), door_pos_prev_.first.first.x, door_pos_prev_.second.first.x);


    //フレーム間距離の引き継ぎ
    min_diff_prev_ = min_diff;
  }
  else if (door_frame_data_updated.size() == 1 && state_ == -1)
  {
    door_pos_prev_.first.first.x = std::get<0>(door_frame_data_updated[0]).x;
    door_pos_prev_.first.first.y = result_img.rows / 2;
    door_pos_prev_.second.first.x = std::get<0>(door_frame_data_updated[0]).x;
    door_pos_prev_.second.first.y = result_img.rows / 2;

    door_state_factor_[0] = 0;
    // RCLCPP_ERROR(this->get_logger(), "unknown");
  }
  else if (door_frame_data_updated.size() == 1 && state_ == 1)
  {
    door_state_factor_[0] = 0;
    // RCLCPP_ERROR(this->get_logger(), "unknown");
  }
  else if (door_frame_data_.empty())
  {
    door_state_factor_[0] = 0;
    // RCLCPP_ERROR(this->get_logger(), "unknown");
  }


  //過去10フレームで連続で開閉どちらかの状態が続いた場合にstateをアップデートする。(開：1, 閉：-1)
  auto it_open = std::search_n(door_state_factor_.begin(), door_state_factor_.end(), 5, 1);
  if (it_open != door_state_factor_.end()) {
      state_ = 1; // 開いている途中
  }

  auto it_close = std::search_n(door_state_factor_.begin(), door_state_factor_.end(), 5, -1);
  if (it_close != door_state_factor_.end()) {
      state_ = -1; // 開いている途中
  }
  // if (std::all_of(door_state_factor_.begin(), door_state_factor_.end(), [](int i){return i == 1;})) {
  //     state_ = 1;   //開いている途中
  // }
  // else if (std::all_of(door_state_factor_.begin(), door_state_factor_.end(), [](int i){return i == -1;})) {
  //     state_ = -1;  //閉まっている途中
  // }
  // RCLCPP_INFO(this->get_logger(), "state:%d", state_);

  //ドア中心フレームの描画
  cv::line(result_img, cv::Point(door_pos.first.first.x, 0), cv::Point(door_pos.first.first.x, result_img.rows - 1), cv::Scalar(0,255,0), 3);
  cv::line(result_img, cv::Point(door_pos.second.first.x, 0), cv::Point(door_pos.second.first.x, result_img.rows - 1), cv::Scalar(0,255,0), 3);


  //openingとclosingの表示
  if (state_ == 1)
  {
    std::string text = "Opening...";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 2.0;
    int thickness = 3;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Scalar color(0, 0, 255); 
    cv::Point textOrg((result_img.cols - textSize.width) / 2, (result_img.rows + textSize.height) / 2 - 100);
    cv::putText(result_img, text, textOrg, fontFace, fontScale, color, thickness);
  }
  else if (state_ == -1)
  {
    std::string text = "Closing...";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 2.0;
    int thickness = 3;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Scalar color(255, 255, 0); 
    cv::Point textOrg((result_img.cols - textSize.width) / 2, (result_img.rows + textSize.height) / 2 - 100);
    cv::putText(result_img, text, textOrg, fontFace, fontScale, color, thickness);
  }


  door_pos.first.first.x = door_pos_prev_.first.first.x;
  door_pos.first.first.y = result_img.rows / 2;
  door_pos.second.first.x = door_pos_prev_.second.first.x;
  door_pos.second.first.y = result_img.rows / 2;
  doorAvePosCalc(depth_img, door_pos);

  
  //ドアのフレーム間の距離を計算
  // door_ave_pos_calc(depth_img, door_pos);
  float x1 = door_pos.first.second.x;
  float y1 = door_pos.first.second.y;
  float z1 = door_pos.first.second.z;
  float x2 = door_pos.second.second.x;
  float y2 = door_pos.second.second.y;
  float z2 = door_pos.second.second.z;
  float dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));


  //開閉判断をする
  if (std::all_of(door_state_factor_.begin(), door_state_factor_.end(), [](int i){return i == 0;}))
  {
    if (state_ != 0)
    {
      if (state_ == 1)
      {
        is_door_opened_ = true;
      }
      else if (state_ == -1)
      {
        is_door_opened_ = false;
      }
    }
    state_ = 0;

    if (is_door_opened_ == true)
    {
      std::string text = "Opened!";
      int fontFace = cv::FONT_HERSHEY_SIMPLEX;
      double fontScale = 2.0;
      int thickness = 3;
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
      cv::Scalar color(0, 0, 255); 
      cv::Point textOrg((result_img.cols - textSize.width) / 2, (result_img.rows + textSize.height) / 2 - 100);
      cv::putText(result_img, text, textOrg, fontFace, fontScale, color, thickness);

      std_msgs::msg::Int32 msg;
      msg.data = 1;
      event_flag_pub_->publish(msg);
    }
    else if (is_door_opened_ == false)
    {
      std::string text = "Closed!";
      int fontFace = cv::FONT_HERSHEY_SIMPLEX;
      double fontScale = 2.0;
      int thickness = 3;
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
      cv::Scalar color(255, 255, 0); 
      cv::Point textOrg((result_img.cols - textSize.width) / 2, (result_img.rows + textSize.height) / 2 - 100);
      cv::putText(result_img, text, textOrg, fontFace, fontScale, color, thickness);
    }
  }
  // RCLCPP_INFO(this->get_logger(), "is_door_opened: %s", is_door_opened_ ? "true" : "false");


  if (img_check)
  {
    // cv::line(result_img, cv::Point(door_pos.first.first.x, 0), cv::Point(door_pos.first.first.x, result_img.rows - 1), cv::Scalar(0,255,0), 3);
    // cv::line(result_img, cv::Point(door_pos.second.first.x, 0), cv::Point(door_pos.second.first.x, result_img.rows - 1), cv::Scalar(0,255,0), 3);

    // ドア中心フレーム間距離の表示
    std::stringstream stream;
    stream << std::fixed << std::setprecision(3) << dis;

    std::string text = stream.str() + " [m]";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 2.0;
    int thickness = 3;
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Scalar color(255, 255, 255); 
    cv::Scalar color1(0,0,0);
    cv::Point textOrg((result_img.cols - textSize.width) / 2, (result_img.rows + textSize.height) / 2);
    cv::putText(result_img, text, textOrg, fontFace, fontScale, color, thickness);

    //ドア中心フレーム間の矢印の描画
    cv::arrowedLine(result_img, cv::Point(door_pos.first.first.x, 350), cv::Point(door_pos.second.first.x, 350), cv::Scalar(255,255,255), 3);
    cv::arrowedLine(result_img, cv::Point(door_pos.second.first.x, 350), cv::Point(door_pos.first.first.x, 350), cv::Scalar(255,255,255), 3);

    cv::imshow("result: calculate_center_frame_distance", result_img);
  }
}  

void DoorServer::doorAvePosCalc(
    cv::Mat& src_img,
    std::pair<std::pair<cv::Point2f, cv::Point3f>, std::pair<cv::Point2f, cv::Point3f>>& door_pos)
{
  int y_pix = (int)door_pos.first.first.y;
  int x_pix = (int)door_pos.first.first.x;
  int near=5;
  int n1 = 0, n2 = 0;
  cv::Point3f point3D1,point3D2;

  if ( y_pix >= near && y_pix < src_img.rows-1-near && x_pix >=near && x_pix < src_img.cols-1-near){

    //const float* src_img_ptr = src_img.ptr<float>(y_pix);				

    //if (src_img_ptr[x_pix] > 0.0)																					
    {
      float depth_sum = 0.0;
      float depth_ave = 0.0;
      float depth = 0.0;
      
      for(int i=-near; i<=near; i++){
        const float* src_img_ptr = src_img.ptr<float>(y_pix+i);				
        for(int j=-near; j<=near; j++){
          if(src_img_ptr[x_pix+j] > 0.0 && src_img_ptr[x_pix+j] < 5000.0){
            depth = src_img_ptr[x_pix+j]*0.001;//m単位
            depth_sum += depth;
            n1++;
          }
        }
      }
      if(n1 > 0) depth_ave = depth_sum/n1;
      pixelToPoint(x_pix, y_pix, depth_ave, door_pos.first.second);

    }
            
  }

  y_pix = (int)door_pos.second.first.y;
  x_pix = (int)door_pos.second.first.x;

  if ( y_pix >= near && y_pix < src_img.rows-1-near && x_pix >=near && x_pix < src_img.cols-1-near){

    //const float* src_img_ptr = src_img.ptr<float>(y_pix);				

    //if (src_img_ptr[x_pix] > 0.0)																					
    {
      float depth_sum = 0.0;
      float depth_ave = 0.0;
      float depth = 0.0;
      
      for(int i=-near; i<=near; i++){
        const float* src_img_ptr = src_img.ptr<float>(y_pix+i);				
        for(int j=-near; j<=near; j++){
          if(src_img_ptr[x_pix+j] > 0.0 && src_img_ptr[x_pix+j] < 5000.0){
            depth = src_img_ptr[x_pix+j]*0.001;//m単位
            depth_sum += depth;
            n2++;
          }
        }
      }
      if(n2 > 0) depth_ave = depth_sum/n2;
      pixelToPoint(x_pix, y_pix, depth_ave, door_pos.second.second);
    }
  }
}

void DoorServer::pixelToPoint(
  float x_pix,
  float y_pix,
  float depth,
  cv::Point3f& point)
{
  //realsese ros width:848 height:480
  //realsense windows width:1280 height:720

  float f;
  //float x = (x_pix - 640.2595) / 644.9059;
  //float y = (y_pix - 352.2614) / 644.9059;
  float x = (x_pix/0.6625 - 640.2595) / 644.9059;
  float y = (y_pix/0.666667 - 352.2614) / 644.9059;
  
  //float x = (x_pix - 424.1719) / 427.2502;
  //float y = (y_pix - 234.8409) / 429.9373;
  //float x = (x_pix/0.6625 - 424.1719) / 427.2502;
  //float y = (y_pix/0.666667 - 234.8409) / 429.9373;
  float ux, uy;
  float coeffs[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

  float r2 = x * x + y * y;
  f = 1 + coeffs[0] * r2 + coeffs[1] * r2 * r2 + coeffs[4] * r2 * r2 * r2;
  ux = x * f + 2 * coeffs[2] * x * y + coeffs[3] * (r2 + 2 * x * x);
  uy = y * f + 2 * coeffs[3] * x * y + coeffs[2] * (r2 + 2 * y * y);
  x = ux;
  y = uy;

  point.x = depth * x;
  point.y = depth * y;
  point.z = depth;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DoorServer)