#include "iostream"
#include <array>
// opencv
#include "opencv4/opencv2/opencv.hpp"

// std
using namespace std;
// opencv
using namespace cv;

class PnPSolver
{
public:
    PnPSolver();
    bool solverPnP(Point2f lift_top, Point2f lift_bottom, Point2f right_top, Point2f right_bottom, Mat &rvec, Mat &tvec);
    float calculateDistanceToCenter(const Point2f &image_point);
    // 相机内参
    Mat camera_matrix_ = (Mat_<double>(3, 3) << 1034.174789, 0.000000, 638.103833,
                          0.000000, 1034.712789, 494.576371,
                          0.000000, 0.000000, 1.000000);
    Mat dist_coeffs_ = (Mat_<double>(1, 5) << -0.100831, 0.094629, -0.000572, 0.000629, 0.000000);
    vector<Point3f> small_armor_points_;

private:
    // mm
    static constexpr float SMALL_ARMOR_WIDTH = 230;
    static constexpr float SMALL_ARMOR_HEIGHT = 127;
};