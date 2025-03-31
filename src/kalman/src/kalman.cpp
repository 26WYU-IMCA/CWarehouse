#include "/home/zz/image/src/kalman/include/kalman/kal.hpp"

void Kal::init(int DP_, int MP_, int CP_)
{
    assert(DP_ > 0 && MP_ > 0);
    CP_ = std::max(CP_, 0);
    this->DP_ = DP_;
    this->MP_ = MP_;
    this->CP_ = CP_;


}
// 初始化参数 模型初始化
void Kal::singer_init(float alpha, float dt, float p, float k, float r)
{

    // A
    transMat << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
        0, 1, (1 - exp(-alpha * dt)) / alpha,
        0, 0, exp(-alpha * dt);
    // H
    measureMat << 1, 0, 0;
    // B
    contrMat << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
        dt - (1 - exp(-alpha * dt) / alpha),
        1 - exp(-alpha * dt);
    // P

    errorCoOre << p, 0, 0,
        0, p, 0,
        0, 0, p;
    // Q
    float q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
    float q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
    float q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
    float q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
    float q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
    float q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
    // K

    processNoiseCov << k * alpha * q11, k * alpha * q12, k * alpha * q13,
        k * alpha * q12, k * alpha * q22, k * alpha * q23,
        k * alpha * q13, k * alpha * q23, k * alpha * q33;
    // R

    measureNoiseCov = r * MatrixXf::Identity(1, 1);
}

// 卡尔曼预测
void Kal::kalman_predict()
{
    if (CP_ > 0)
        statePre = transMat * stateOre + contrMat * stateOre(2, 0);
    else
        statePre = transMat * stateOre;

    errorCoPre = transMat * errorCoOre * transMat.transpose() + processNoiseCov;
}
// 更新
void Kal::kalman_updata()
{
    kgain = errorCoPre * measureMat.transpose() * (measureMat * errorCoPre * measureMat.transpose() + measureNoiseCov).inverse();
    stateOre = statePre + kgain * (measurement - measureMat * statePre);
    errorCoOre = (MatrixXf::Identity(DP_, DP_) - kgain * measureMat) * errorCoPre;
}

void Kal::kfinit_uniform()
{
    IT_ = 1;
    processNoiseCov << 1e-6f, 0, 0,
        0, 1e-6f, 0,
        0, 0, 1e-6f;
    measureNoiseCov = MatrixXf::Identity(1, 1) * 1e-2f;
    transMat << 1, IT_, IT_ * IT_ / 2,
        0, 1, IT_,
        0, 0, 1;
    measureMat << 1, 0, 0;
    errorCoOre << 1e-2f, 0, 0,
        0, 1e-2f, 0,
        0, 0, 1e-2f;
}

bool Kal::same_armor()
{
    if (abs(last_armor.positiom_y - new_armor.positiom_y) < 0.3)
        return true;
    else
        return false;
}