// std
#include <iostream>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// opencv
#include <opencv4/opencv2/opencv.hpp>

// std
using namespace std;

// opencv
using namespace cv;

class PubImage : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

public:
  PubImage() : Node("pub_image")
  {
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    Mat img;
    cv::VideoCapture cap("/home/zz/opencv_cpp/src/1.mp4");
    while (1)
    {
      cap >> img;
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "camera";
      auto src = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
      pub_image_->publish(*src);
      rclcpp::sleep_for(7ms);
    }
  }
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PubImage>();
  rclcpp::shutdown();
  return 0;
}