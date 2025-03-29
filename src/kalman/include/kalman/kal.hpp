//std
#include <iostream>
using namespace std;
//opencv
#include <opencv4/opencv2/opencv.hpp>
using namespace cv;

class Kal
{
public:
    struct Armor
    {
        float armor_angle;
        float armor_speed;
    //mm
        static constexpr float ARMOR_WIDTH = 230;
        static constexpr float ARMOR_HEIGHT = 127;
    //相机坐标
        float positiom_x;
        float positiom_y;
        float positiom_z;
    //图像坐标
        float x;
        float y;
    //预测坐标
        float pre_x;
        float pre_y;
    //到中心点距离
        float d_center = 0;
    }last_armor,new_armor;
//函数声明
    bool same_armor();
    void init(int DP_ ,int MP_ ,int CP_);
    void singer_init(float alpha ,float dt ,float p ,float k ,float r);
    void kalman_predict();
    void kalman_updata(Mat measureMat_last);
    void kfinit_uniform();
    void kfinit_singer();
//变量定义
    bool update_armor = 0;
    float singer[5];
    int DP_;
    int MP_;
    int CP_;
    Mat statePre;
    Mat stateOre;
    Mat transMat;
    Mat measureMat;
    Mat processNoiseCov;
    Mat measureNoiseCov;
    Mat errorCoPre;
    Mat errorCoOre;
    Mat kgain;
    Mat contrMat;
    cv::KalmanFilter kalman;
    int rec_x = 0;
};
