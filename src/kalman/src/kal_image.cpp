// std
#include <iostream>

// opencv
#include <opencv4/opencv2/opencv.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>
#include "base_interfaces_demo/msg/armor.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
// kal
#include "/home/zz/image/src/kalman/include/kalman/kal.hpp"

//eigen
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace cv;
using namespace Eigen;
class KalmanNode : public rclcpp::Node
{
private:
  // 定义变量
  Point2f center;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<base_interfaces_demo::msg::Armor>::SharedPtr x_y_z_;
  void callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void get_xyz(const base_interfaces_demo::msg::Armor::ConstSharedPtr &msg);
  std::shared_ptr<Kal> kalman_;

public:
  KalmanNode() : Node("rec_image")
  {
    x_y_z_ = this->create_subscription<base_interfaces_demo::msg::Armor>("camera/xyz", 10, std::bind(&KalmanNode::get_xyz, this, std::placeholders::_1));
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>("image1", 10, std::bind(&KalmanNode::callback, this, std::placeholders::_1));
    kalman_ = std::make_shared<Kal>();
    kalman_->init(3, 1, 1);
    kalman_->singer_init(5, 5, 1, 0.01, 1);
  }
};
void KalmanNode::get_xyz(const base_interfaces_demo::msg::Armor::ConstSharedPtr &msg)
{

  if (kalman_->update_armor == 0)
  {
    kalman_->new_armor.x = msg->cx;
    kalman_->new_armor.y = msg->cy;
    kalman_->new_armor.positiom_x = msg->px;
    kalman_->new_armor.positiom_y = msg->py;
    kalman_->new_armor.positiom_z = msg->pz;
    kalman_->last_armor = kalman_->new_armor;
    kalman_->stateOre << kalman_->new_armor.y, 0, 0;
    kalman_->update_armor = 1;
  }
  else
  {
    kalman_->last_armor = kalman_->new_armor;
    kalman_->new_armor.x = msg->cx;
    kalman_->new_armor.y = msg->cy;
    kalman_->new_armor.positiom_x = msg->px;
    kalman_->new_armor.positiom_y = msg->py;
    kalman_->new_armor.positiom_z = msg->pz;
    kalman_->new_armor.armor_speed = kalman_->stateOre(1);
    if (!kalman_->same_armor())
    {
      kalman_->last_armor.x = msg->cx;
      kalman_->last_armor.y = msg->cy;
      kalman_->last_armor.positiom_x = msg->px;
      kalman_->last_armor.positiom_y = msg->py;
      kalman_->last_armor.positiom_z = msg->pz;
      kalman_->stateOre(0) = kalman_->new_armor.x;
      kalman_->stateOre(1) = 0;
    }
  }
  kalman_->new_armor.pre_y = kalman_->new_armor.y;
  kalman_->measurement << kalman_->new_armor.x;
  kalman_->kalman_predict();
  kalman_->kalman_updata();
}
void KalmanNode::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{

  cv::Mat src;
  auto src_ = cv_bridge::toCvShare(msg, "bgr8");
  src = src_->image;

  if (kalman_->stateOre(0) != 0)
  {
    cv::circle(src, cv::Point2f(kalman_->stateOre(0), kalman_->new_armor.pre_y), 5, cv::Scalar(0, 0, 255), -1);
  }

  cv::imshow("video", src);
  cv::waitKey(10);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
