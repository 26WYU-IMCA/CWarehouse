#include "/home/zz/image/src/recognition_image/include/rec.hpp"

void Rec::pretreatment(Mat image_, Mat &PreImage)
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

void Rec::PreliminaryIdentification(Mat &PreImage_, vector<vector<Point>> &contours, Rect point_array[20], int &index)
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
        point_array[index] = bountRect;
        index++; // 增加匹配矩形的数量
      }
    }
  }
}

void Rec::SecondaryScreening(Rect point_array[], int index, int point_near[], int &min)
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

bool Rec::secondary_screening(Rect point_array[], int point_near[])
{
  return point_array[point_near[0]].area() > 150 && point_array[point_near[1]].area() > 150 && abs(point_array[point_near[0]].y - point_array[point_near[1]].y) < 11 && abs(point_array[point_near[0]].x - point_array[point_near[1]].x) < 130;
}

bool Rec::effectiveness(Rect rectangle_1, Rect rectangle_2)
{
  return rectangle_1.x == 0 || rectangle_2.x == 0;
}

void Rec::Draw_recognition(Rect point_array[], int point_near[], Point2d &center, Point2f point_[])
{
  if (secondary_screening(point_array, point_near)) // 二次筛选
  {
    Rect rectangle_1 = point_array[point_near[0]];
    Rect rectangle_2 = point_array[point_near[1]];
    if (effectiveness(rectangle_1, rectangle_2))
    {
      throw "not"; // 若无效则抛出异常
    }
    else
    {
      // 计算矩形的中心点
      Point point1 = Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y);
      Point point2 = Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y + rectangle_1.height);
      Point point3 = Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y);
      Point point4 = Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y + rectangle_2.height);
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
