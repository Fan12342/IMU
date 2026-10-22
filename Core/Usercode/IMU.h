//
// Created by Administrator on 2026/10/22.
//

#ifndef IMA_IMU2_H
#define IMA_IMU2_H
#include <stdint.h>

#include "pid.h"

class IMU
{
    public:
    void update(uint32_t dt_ms);


    // dma 读取相关
    enum class dma_states
    {
        IDLE,
        READING_ACCEL,
        READING_GYRO,
        READING_TEMP
    } dma_state_ = dma_states::IDLE;
    uint32_t update_dt_ms_;
    void dmaCallback();




    // acc 相关
    float acc_data_to_mg_;
    uint8_t acc_rx_[8];
    float acc_raw_data_[3];
    float acc_transformed_data_[3];
    float acc_bias_[3];
    float acc_filtered_data_[3];

    void acc_init();
    void acc_read_dma();
    void acc_cal(uint8_t* data);




    // gyro 相关
    float gyro_data_to_dps_;
    uint8_t gyro_rx_[7];
    float gyro_raw_data_[3];
    float gyro_transformed_data_[3];
    float gyro_bias_[3];
    float gyro_filtered_data_[3];

    void gyro_init();
    void gyro_read_dma();
    void gyro_cal(uint8_t* data);
    void gyro_calibrate(uint32_t time_ms);



    // 数据处理
    const float R_imu_[3][3] = {
        {-1.0f, 0.0f, 0.0f},
        {0.0f, -1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    bool is_first_ = true;
    float filter_k_ = 0.3f;

    // 姿态四元数，顺序为 w、x、y、z
    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // 姿态角
    float roll_  = 0.0f;
    float pitch_ = 0.0f;
    float yaw_   = 0.0f;

    // Mahony
    float mahony_kp_ = 1.0f;
    float mahony_ki_ = 0.0f;

    float ex_sum = 0.0f, ey_sum = 0.0f, ez_sum = 0.0f;

    void data_transform();
    void data_filter();
    void attitude_update(uint32_t dt_ms);



    // 温控
    class TempControl
    {
    public:
        float temp_ = 0;
        float temp_ref_ = 27.0f;
        PID pid_ = PID(4000, 0, 0, 0, 999, 0);
        uint32_t last_tick;
        uint8_t temp_data[4];

        void read_temp();
        void read_temp_block();
        void handle();// 输出到 PWM
        void transform_data(uint8_t* data);
        bool try_control();
        void temp_control(uint16_t time_ms);// 初始化控温

    private:
    } temp_control_;
};

#endif //IMA_IMU2_H
