//
// Created by Administrator on 2026/10/18.
//

#ifndef IMA_IMU_H
#define IMA_IMU_H
#include "bmi088.h"
#include "pid.h"

class IMU
{
public:
    class TempControl
    {
    public:
        float temperature_ = 0;
        float temp_ref_ = 29.0f;
        PID pid_ = PID(4000, 0, 0, 0, 999, 0);
        uint16_t output_;

        void read_temperature();
        void temp_control(uint16_t time_ms);
        void handle();

    private:
    };

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
    float gyro_filter_[3], accel_filter_[3];
    bool is_first_ = true;

    float filter_k_ = 0.5f;

    float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
    float accel_bias_[3] = {0.0f, 0.0f, 0.0f};

    // 加速度计的比例修正系数
    float accel_sen_ = 1.0f;

    TempControl temp_control_;


    // 姿态四元数，顺序为 w、x、y、z
    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // 姿态角，单位：度
    float roll_  = 0.0f;
    float pitch_ = 0.0f;
    float yaw_   = 0.0f;

    // Mahony 比例校正系数
    float mahony_kp_ = 1.0f;
    float mahony_ki_ = 0.0f;

    float ex_sum = 0.0f, ey_sum = 0.0f, ez_sum = 0.0f;

    // 姿态解算
    void attitude_update(float dt_s);

    void acc_calculate();
    void gyro_calculate();

    void acc_init();
    void gyro_init();

    void sensorDataTransform();

    void dataFilter();

    void update(float dt_s);

    void gyro_calibrate(uint32_t time_ms);

    // private:
};

#endif //IMA_IMU_H
