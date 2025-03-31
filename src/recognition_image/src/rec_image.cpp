
// std
#include <iostream>

// ros
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

// opencv
#include "opencv4/opencv2/opencv.hpp"

// HPP
#include </home/zz/image/src/recognition_image/include/pnp.hpp>
#include </home/zz/image/src/recognition_image/include/rec.hpp>
#include "base_interfaces_demo/msg/armor.hpp"
// std
using namespace std;

// opencv
using namespace cv;

class RecImage : public rclcpp::Node
{
private:
  // 定义变量
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<base_interfaces_demo::msg::Armor>::SharedPtr x_y_z_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  // 声明函数
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void send_x_y_z(Point2f point_[], Mat &rvec, Mat &tvec, Point2d center);
  void pretreatment(Mat image_, Mat &PreImage_);
  void PreliminaryIdentification(Mat &PreImage_,
                                 vector<vector<Point>> &contours, Rect point_array[], int &index);
  void SecondaryScreening(Rect point_array[], int index, int point_near[], int &min);
  bool secondary_screening(Rect point_array[], int point_near[]);
  bool effectiveness(Rect rectangle_1, Rect rectangle_2);
  void Draw_recognition(Rect point_array[], int point_near[], Point2d &center, Point2f point_[]);

  // 类
  std::shared_ptr<PnPSolver> pnp_solver_;
  // std::shared_ptr<Rec> rec_;
public:
  RecImage() : Node("rec_image")
  {

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&RecImage::callback, this, std::placeholders::_1));
    x_y_z_ = this->create_publisher<base_interfaces_demo::msg::Armor>("camera/xyz", 10);
    pnp_solver_ = std::make_shared<PnPSolver>();

    // rec_ = std::make_shared<Rec>();
  }
};
void RecImage::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv::Mat image_;
  cv::Mat PreImage_;
  Rect point_array[100];          // 存放可能的矩形区域
  int index = 0;                  // 当前存储的矩形数量
  int point_near[2];              // 存放距离最近的矩形的序号
  int min = 100000;               // 存放距离最近的矩形的距离
  vector<vector<Point>> contours; // 用于存放轮廓
  Point2d center;
  Point2f point_[4];
  Mat rvec, tvec;
  auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  image_ = cv_ptr->image;
  pretreatment(image_, PreImage_);
  PreliminaryIdentification(PreImage_, contours, point_array, index);
  SecondaryScreening(point_array, index, point_near, min);
  Draw_recognition(point_array, point_near, center, point_);
  send_x_y_z(point_, rvec, tvec, center);
  circle(image_, center, 5, Scalar(255, 0, 0), -1);
  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image1", 10);
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "camera";
  auto src = cv_bridge::CvImage(header, "bgr8", image_).toImageMsg();
  pub_image_->publish(*src);
  rclcpp::sleep_for(7ms);
  // imshow("PreImage", image_);
  // waitKey(10);
}

void RecImage::send_x_y_z(Point2f point_[], Mat &rvec, Mat &tvec, Point2d center)
{
  float px, py, pz;

  bool success = pnp_solver_->solverPnP(point_[0], point_[1], point_[2], point_[3], rvec, tvec);

  if (success)
  {

    px = tvec.at<double>(0);
    py = tvec.at<double>(1);
    pz = tvec.at<double>(2);
    Mat rotation_matric;
    cv::Rodrigues(rvec, rotation_matric);
    tf2::Matrix3x3 tf2_rotation_mateic(
        rotation_matric.at<double>(0, 0), rotation_matric.at<double>(0, 1), rotation_matric.at<double>(0, 2),
        rotation_matric.at<double>(1, 0), rotation_matric.at<double>(1, 1), rotation_matric.at<double>(1, 2),
        rotation_matric.at<double>(2, 0), rotation_matric.at<double>(2, 1), rotation_matric.at<double>(2, 2));

    tf2::Quaternion tf2_q;
    tf2_rotation_mateic.getRotation(tf2_q);
    auto msg = std::make_shared<base_interfaces_demo::msg::Armor>();
    msg->cx = center.x;
    msg->cy = center.y;
    msg->px = px;
    msg->py = py;
    msg->pz = pz;
    x_y_z_->publish(*msg);
  }
}
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecImage>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void RecImage::pretreatment(Mat image_, Mat &PreImage)
{
  Mat image_copy = image_; // 备份当前帧
  vector<Mat> separate;    // 用于存放通道分离的结果

  // 通道分离
  split(image_copy, separate);
  Mat binarized; // 二值化后的对象
  // 二值化处理
  threshold(separate[0], binarized, 150, 255, 0);
  // 应用高斯模糊
  GaussianBlur(binarized, PreImage, Size(5, 5), 0);
}

void RecImage::PreliminaryIdentification(Mat &PreImage_, vector<vector<Point>> &contours, Rect point_array[20], int &index)
{
  findContours(PreImage_, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  for (size_t i = 0; i < contours.size(); i++)
  {
    RotatedRect box = minAreaRect(Mat(contours[i]));
    vector<Point2f> boxp(4);
    box.points(boxp.data()); // 获取矩形的四个点
    if (box.size.area() > 250)
    {
      // 获取轮廓的边界矩形
      Rect bountRect = boundingRect(Mat(contours[i]));

      // 根据条件筛选合适的矩形
      if (double(bountRect.height / bountRect.width) >= 1.3 && bountRect.height > 15)
      {
        // rectangle(image_, bountRect, Scalar(0, 255, 0), 2, 8, 0);
        point_array[index] = bountRect;
        index++; // 增加匹配矩形的数量
      }
    }
  }
}

void RecImage::SecondaryScreening(Rect point_array[], int index, int point_near[], int &min)
{
  for (int i = 0; i < index - 1; i++)
  {
    for (int j = i + 1; j < index; j++)
    {

      int value = abs(point_array[i].y - point_array[j].y);
      if (min > value)
      {
        min = value;       // 更新最小值
        point_near[0] = i; // 存储最接近的两个矩形的索引
        point_near[1] = j;
      }
    }
  }
}

bool RecImage::secondary_screening(Rect point_array[], int point_near[])
{
  return point_array[point_near[0]].area() > 150 && point_array[point_near[1]].area() > 150 && abs(point_array[point_near[0]].y - point_array[point_near[1]].y) < 11 && abs(point_array[point_near[0]].x - point_array[point_near[1]].x) < 130;
}

bool RecImage::effectiveness(Rect rectangle_1, Rect rectangle_2)
{
  return rectangle_1.x == 0 || rectangle_2.x == 0;
}

void RecImage::Draw_recognition(Rect point_array[], int point_near[], Point2d &center, Point2f point_[])
{
  if (point_array[point_near[0]].area() > 150 && point_array[point_near[1]].area() > 150 && abs(point_array[point_near[0]].y - point_array[point_near[1]].y) < 11 && abs(point_array[point_near[0]].x - point_array[point_near[1]].x) < 130) // 二次筛选
  {
    Rect rectangle_1 = point_array[point_near[0]];
    Rect rectangle_2 = point_array[point_near[1]];
    if (!effectiveness(rectangle_1, rectangle_2))
    {
      // 计算矩形的中心点
      Point2f point1 = Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y);
      Point2f point2 = Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y + rectangle_1.height);
      Point2f point3 = Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y);
      Point2f point4 = Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y + rectangle_2.height);
      // 存储四个顶点
      point_[0] = point1; // 左上
      point_[1] = point2; // 左下
      point_[2] = point3; // 右上
      point_[3] = point4; // 右下

      center.x = (point1.x + point2.x + point3.x + point4.x) / 4;
      center.y = (point1.y + point2.y + point3.y + point4.y) / 4;
    }
  }
}
