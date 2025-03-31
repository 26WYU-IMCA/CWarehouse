//std
#include <iostream>
//opencv
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;
using namespace std;
class Rec
{
    public:  
  void pretreatment(Mat image_, Mat &PreImage_);
  void PreliminaryIdentification(Mat &PreImage_,
  vector<vector<Point>> &contours,Rect point_array[20],int &index);
  void SecondaryScreening(Rect point_array[], int index, int point_near[], int &min);
  bool secondary_screening(Rect point_array[], int point_near[]);
  bool effectiveness(Rect rectangle_1, Rect rectangle_2);
  void Draw_recognition(Rect point_array[], int point_near[], Point2d &center, Point2f point_[]);

};