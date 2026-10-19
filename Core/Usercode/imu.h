//
// Created by Administrator on 2026/10/18.
//

#ifndef IMA_IMU_H
#define IMA_IMU_H
#include "bmi088.h"

class IMU
{
public:
    const float R_imu_[3][3] = {
        {-1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, -1.0f}
    };

    struct Raw_data
    {
        float accel[3];
        float gyro[3];
    } raw_data_;

    float acc_data_to_mg_;
    float gyro_data_to_dps_;

    // 机体坐标系数据
    float gyro_sensor_[3], accel_sensor_[3];

    float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
    float accel_bias_[3] = {0.0f, 0.0f, 0.0f};

    // 加速度计的比例修正系数
    float accel_sen_ = 1.0f;

    void acc_calculate();
    void gyro_calculate();

    void acc_init();
    void gyro_init();

    void sensorDataTransform();

    void update();

    // private:

};

#endif //IMA_IMU_H
