#include "/home/zz/image/src/kalman/include/kalman/kal.hpp"

void Kal::init(int DP_, int MP_, int CP_)
{
    statePre = cv::Mat::zeros(DP_, 1, CV_32F); // x'
    stateOre = cv::Mat::zeros(DP_, 1, CV_32F); // x
    kalman.init(DP_, MP_, CP_, CV_32F);
}
// 初始化参数
void Kal::singer_init(float alpha, float dt, float p, float k, float r)
{
    singer[0] = alpha;
    singer[1] = dt;
    singer[2] = p;
    singer[3] = k;
    singer[4] = r;
}
// 模型初始化
void Kal::kfinit_singer()
{
    float alpha = singer[0];
    float dt = singer[1];
    // A
    kalman.transitionMatrix = (cv::Mat_<float>(3, 3) << 
    1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
    0, 1, (1 - exp(-alpha * dt)) / alpha,
    0, 0, exp(-alpha * dt));
    // H
    kalman.measurementMatrix = (cv::Mat_<float>(1, 3) << 1, 0, 0);
    // B
    kalman.controlMatrix = (cv::Mat_<float>(3, 1) << 
    1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
    dt - (1 - exp(-alpha * dt) / alpha),
    1 - exp(-alpha * dt));
    //P
    float p = singer[2];
    kalman.errorCovPost = (cv::Mat_<float>(3,3) << 
    p, 0, 0,
    0, p, 0,
    0, 0, p);
    //Q
    float q11 = 1/(2*pow(alpha,5))*(1-exp(-2*alpha*dt)+2*alpha*dt+2*pow(alpha*dt,3)/3-2*pow(alpha*dt,2)-4*alpha*dt*exp(-alpha*dt));
    float q12 = 1/(2*pow(alpha,4))*(exp(-2*alpha*dt)+1-2*exp(-alpha*dt)+2*alpha*dt*exp(-alpha*dt)-2*alpha*dt+pow(alpha*dt,2));
    float q13 = 1/(2*pow(alpha,3))*(1-exp(-2*alpha*dt)-2*alpha*dt*exp(-alpha*dt));
    float q22 = 1/(2*pow(alpha,3))*(4*exp(-alpha*dt)-3-exp(-2*alpha*dt)+2*alpha*dt);
    float q23 = 1/(2*pow(alpha,2))*(exp(-2*alpha*dt)+1-2*exp(-alpha*dt));
    float q33 = 1/(2*alpha)*(1-exp(-2*alpha*dt));
    //K
    float k = singer[3];
    kalman.processNoiseCov = (cv::Mat_<float>(3,3) << 
    k*alpha*q11, k*alpha*q12, k*alpha*q13,
    k*alpha*q12, k*alpha*q22, k*alpha*q23,
    k*alpha*q13, k*alpha*q23, k*alpha*q33);
    //R
    float r = singer[4];
    kalman.measurementNoiseCov = r*cv::Mat::eye(1,1,CV_32F);
}
//卡尔曼预测
void Kal::kalman_predict()
{
    statePre = kalman.predict();
}
//更新
void Kal::kalman_updata(Mat measureMat_last)
{
    stateOre = kalman.correct(measureMat_last);
}

void Kal::kfinit_uniform()
{
    float dt = 0.8;
    float process_noise = 1e-6f;
    float measurement_noise = 1e-2f;
    float error_covariance = 1e-2f;

    kalman.transitionMatrix = (cv::Mat_<float>(3,3) << 
    1, dt, dt*dt/2,
    0, 1, dt,
    0, 0, 1);
   
    kalman.measurementMatrix = (cv::Mat_<float>(1,3) << 1,0,0);
    //P
    kalman.errorCovPost = cv::Mat::eye(3,3,CV_32F)*error_covariance;
    //Q
    kalman.processNoiseCov = cv::Mat::eye(3,3,CV_32F)*process_noise;
    //R
    kalman.measurementNoiseCov = measurement_noise*cv::Mat::eye(1,1,CV_32F);
}

bool Kal::same_armor()
{
    if(abs(last_armor.positiom_y-new_armor.positiom_y)<0.3)
    return true;
    else
    return false;

}