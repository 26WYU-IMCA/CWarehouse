//std
#include <iostream>
//eigen
#include <eigen3/Eigen/Dense>
//opencv
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

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
    void kalman_updata();
    void kfinit_uniform();
    void kfinit_singer();
//变量定义
    bool update_armor = 0;
    int DP_;
    int MP_;
    int CP_;
    int IT_;
    MatrixXf statePre= MatrixXf::Zero(3, 1);
    MatrixXf stateOre = MatrixXf::Zero(3, 1);
    MatrixXf transMat = MatrixXf::Zero(3, 3);
    MatrixXf measureMat= MatrixXf::Zero(1, 3);
    MatrixXf measurement= MatrixXf::Zero(1, 1);
    MatrixXf processNoiseCov= MatrixXf::Zero(3, 3);
    MatrixXf measureNoiseCov= MatrixXf::Zero(1, 1);
    MatrixXf errorCoPre= MatrixXf::Zero(3, 3);
    MatrixXf errorCoOre= MatrixXf::Zero(3, 3);
    MatrixXf kgain= MatrixXf::Zero(3, 1);
    MatrixXf contrMat= MatrixXf::Zero(3, 1);
};
