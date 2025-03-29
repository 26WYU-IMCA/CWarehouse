#include "/home/zz/image/src/recognition_image/include/pnp.hpp"
#include "opencv4/opencv2/opencv.hpp"
using namespace cv;
using namespace std;

PnPSolver::PnPSolver()
{
    constexpr double small_half_y = SMALL_ARMOR_WIDTH/2000;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT/2000;
    small_armor_points_.emplace_back(Point3f(0 ,-small_half_y ,small_half_z));
    small_armor_points_.emplace_back(Point3f(0 ,small_half_y ,small_half_z));
    //small_armor_points_.emplace_back(Point3f(0 ,small_half_y ,0));
    small_armor_points_.emplace_back(Point3f(0 ,small_half_y ,-small_half_z));
    small_armor_points_.emplace_back(Point3f(0 ,-small_half_y ,-small_half_z));
    //small_armor_points_.emplace_back(Point3f(0 ,-small_half_y ,0));
    
    
    
}

bool PnPSolver::solverPnP(Point2f lift_top, Point2f lift_bottom, Point2f right_top, Point2f right_bottom, Mat &rvec, Mat &tvec)
{
    vector<Point2f> image_armor_points;
    image_armor_points.emplace_back(lift_top); 
    image_armor_points.emplace_back(right_top);
    //image_armor_points.emplace_back(Point2f((right_top.x+right_bottom.x)/2,(right_top.y+right_bottom.y)/2));
    image_armor_points.emplace_back(right_bottom);
    image_armor_points.emplace_back(lift_bottom);
    //image_armor_points.emplace_back(Point2f((lift_top.x+lift_bottom.x)/2,(lift_top.y+lift_bottom.y)/2));
    vector<Point3f> object_points = small_armor_points_;
    bool ret = cv::solvePnP(object_points,image_armor_points,camera_matrix_,dist_coeffs_,rvec,tvec);
    return ret;
}

float PnPSolver::calculateDistanceToCenter(const Point2f &image_point)
{
    float cx = camera_matrix_.at<double>(0,2);
    float cy = camera_matrix_.at<double>(1,2);
    return cv::norm(image_point-cv::Point2f(cx,cy));
}