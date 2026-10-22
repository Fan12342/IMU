//
// Created by Administrator on 2026/10/22.
//

#include "IMU.h"
#include <cmath>
#include "bmi088.h"
#include "tim.h"

void IMU::update(uint32_t dt_ms)
{
    if (dma_state_ == dma_states::IDLE)
    {
        update_dt_ms_ = dt_ms;
        acc_read_dma();
        dma_state_ = dma_states::READING_ACCEL;
    }
}

void IMU::dmaCallback()
{
    switch (dma_state_)
    {
    case dma_states::IDLE:
        break;

    case dma_states::READING_ACCEL:
        gyro_read_dma();
        dma_state_ = dma_states::READING_GYRO;
        break;

    case dma_states::READING_GYRO:
        BMI088_GYRO_NS_H();
        BMI088_ACCEL_NS_H();
        // 跳过第一位指令和第二位 dummy byte
        acc_cal(&acc_rx_[2]);
        // 跳过第一位指令
        gyro_cal(&gyro_rx_[1]);
        data_transform();
        data_filter();
        attitude_update(update_dt_ms_);

        if (temp_control_.try_control())
            dma_state_ = dma_states::READING_TEMP;
        else
            dma_state_ = dma_states::IDLE;
        break;

    case dma_states::READING_TEMP:
        temp_control_.transform_data(&temp_control_.temp_data[2]);
        temp_control_.handle();
        dma_state_ = dma_states::IDLE;
        break;

    default:
        break;
    }
}


// 阻塞式
void IMU::acc_init()
{
    uint8_t raw_range;
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    acc_data_to_mg_ = 1000.0f * 3.0f * static_cast<float>(1U << (raw_range & 0x03)) / 32768.0f;
}

void IMU::acc_read_dma()
{
    static uint8_t tx_buf[8];
    const uint8_t size = 8;

    tx_buf[0] = (uint8_t)(0x12 | 0x80);
    for (uint16_t i = 1U; i < size; i++)
        tx_buf[i] = 0x00;

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, acc_rx_, size);
}

void IMU::acc_cal(uint8_t* data)
{
    int16_t Accel_X = (data[1] << 8) | data[0];
    int16_t Accel_Y = (data[3] << 8) | data[2];
    int16_t Accel_Z = (data[5] << 8) | data[4];

    acc_raw_data_[0] = static_cast<float>(Accel_X) * acc_data_to_mg_;
    acc_raw_data_[1] = static_cast<float>(Accel_Y) * acc_data_to_mg_;
    acc_raw_data_[2] = static_cast<float>(Accel_Z) * acc_data_to_mg_;
}




// 阻塞式
void IMU::gyro_init()
{
    uint8_t raw_range;
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    gyro_data_to_dps_ = 2000.0f / static_cast<float>(1U << (raw_range & 0x07)) / 32768.0f;
}

void IMU::gyro_read_dma()
{
    static uint8_t tx_buf[7];
    const uint8_t size = 7;

    tx_buf[0] = 0x02 | 0x80;
    for (uint16_t i = 1U; i < size; i++)
        tx_buf[i] = 0x00;

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, gyro_rx_, size);
}

void IMU::gyro_cal(uint8_t* data)
{
    int16_t Rate_X = (data[1] << 8) | data[0];
    int16_t Rate_Y = (data[3] << 8) | data[2];
    int16_t Rate_Z = (data[5] << 8) | data[4];

    gyro_raw_data_[0] = static_cast<float>(Rate_X) * gyro_data_to_dps_;
    gyro_raw_data_[1] = static_cast<float>(Rate_Y) * gyro_data_to_dps_;
    gyro_raw_data_[2] = static_cast<float>(Rate_Z) * gyro_data_to_dps_;
}

// 阻塞式
void IMU::gyro_calibrate(uint32_t time_ms)
{
    float raw_data_sum[3] = {0.0f, 0.0f, 0.0f};

    uint32_t samples = 0;

    // 求和
    uint32_t start_tick = HAL_GetTick();
    do
    {
        samples++;
        bmi088_gyro_read_reg(0x02, gyro_rx_, 6);
        gyro_cal(gyro_rx_);
        for (uint8_t i = 0; i < 3; i++)
            raw_data_sum[i] += gyro_raw_data_[i];
        HAL_Delay(1);
    }
    while (HAL_GetTick() < start_tick + time_ms);

    for (uint8_t i = 0; i < 3; i++)
    {
        float sum = 0.0f;
        for (uint8_t j = 0; j < 3; j++)
            sum += R_imu_[i][j] * raw_data_sum[j];
        gyro_bias_[i] = sum / static_cast<float>(samples);
    }
}

void IMU::data_transform()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        acc_transformed_data_[i] = -acc_bias_[i];
        for (uint8_t j = 0; j < 3; j++)
            acc_transformed_data_[i] += R_imu_[i][j] * acc_raw_data_[j];

        gyro_transformed_data_[i] = -gyro_bias_[i];
        for (uint8_t j = 0; j < 3; j++)
            gyro_transformed_data_[i] += R_imu_[i][j] * gyro_raw_data_[j];

    }
}

void IMU::data_filter()
{
    if (is_first_)
    {
        is_first_ = false;
        for (uint8_t i = 0; i < 3; i++)
        {
            acc_filtered_data_[i] = acc_transformed_data_[i];
            gyro_filtered_data_[i] = gyro_transformed_data_[i];
        }
        return;
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        acc_filtered_data_[i] = acc_filtered_data_[i] * (1 - filter_k_) + acc_transformed_data_[i] * filter_k_;
        gyro_filtered_data_[i] = gyro_filtered_data_[i] * (1 - filter_k_) + gyro_transformed_data_[i] * filter_k_;
    }
}

void IMU::attitude_update(uint32_t dt_ms)
{
    const float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;

    // 换成弧度
    float gx = gyro_filtered_data_[0] * DEG_TO_RAD;
    float gy = gyro_filtered_data_[1] * DEG_TO_RAD;
    float gz = gyro_filtered_data_[2] * DEG_TO_RAD;

    // 通过加速度计估计重力方向
    float ax = acc_filtered_data_[0];
    float ay = acc_filtered_data_[1];
    float az = acc_filtered_data_[2];

    float acc_norm = std::sqrt(ax * ax + ay * ay + az * az);

    // 仅在加速度模长接近重力时使用加速度校正
    if (acc_norm > 500.0f && acc_norm < 1500.0f)
    {
        // 归一化
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        // 根据四元数估计重力方向
        float vx = 2.0f * (q_[1] * q_[3] - q_[0] * q_[2]);
        float vy = 2.0f * (q_[0] * q_[1] + q_[2] * q_[3]);
        float vz = 1.0f - 2.0f * (q_[1] * q_[1] + q_[2] * q_[2]);

        // 测量方向 × 估计方向
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        // Mahony 比例反馈
        gx += mahony_kp_ * ex;
        gy += mahony_kp_ * ey;
        gz += mahony_kp_ * ez;

        // Mahony 积分反馈
        ex_sum += ex * dt_s;
        ey_sum += ey * dt_s;
        ez_sum += ez * dt_s;
        gx += mahony_ki_ * ex_sum;
        gy += mahony_ki_ * ey_sum;
        gz += mahony_ki_ * ez_sum;
    }

    // 四元数微分方程
    float q_dot0 = 0.5f * (-q_[1] * gx - q_[2] * gy - q_[3] * gz);
    float q_dot1 = 0.5f * (q_[0] * gx + q_[2] * gz - q_[3] * gy);
    float q_dot2 = 0.5f * (q_[0] * gy - q_[1] * gz + q_[3] * gx);
    float q_dot3 = 0.5f * (q_[0] * gz + q_[1] * gy - q_[2] * gx);

    // 积分更新
    q_[0] += q_dot0 * dt_s;
    q_[1] += q_dot1 * dt_s;
    q_[2] += q_dot2 * dt_s;
    q_[3] += q_dot3 * dt_s;

    // 归一化
    float q_norm = std::sqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);

    if (q_norm < 1e-6f)
        return;

    q_[0] /= q_norm;
    q_[1] /= q_norm;
    q_[2] /= q_norm;
    q_[3] /= q_norm;

    // 四元数转欧拉角，采用 Z-Y-X 顺序
    constexpr float RAD_TO_DEG = 180.0f / 3.14159265f;

    float sin_pitch = 2.0f * (q_[0] * q_[2] - q_[3] * q_[1]);

    // 防止浮点误差使 asin 输入超出 [-1, 1]
    if (sin_pitch > 1.0f)
        sin_pitch = 1.0f;
    else if (sin_pitch < -1.0f)
        sin_pitch = -1.0f;

    roll_ = std::atan2(
        2.0f * (q_[0] * q_[1] + q_[2] * q_[3]),
        1.0f - 2.0f * (q_[1] * q_[1] + q_[2] * q_[2])
    ) * RAD_TO_DEG;

    pitch_ = std::asin(sin_pitch) * RAD_TO_DEG;

    yaw_ = std::atan2(
        2.0f * (q_[0] * q_[3] + q_[1] * q_[2]),
        1.0f - 2.0f * (q_[2] * q_[2] + q_[3] * q_[3])
    ) * RAD_TO_DEG;
}




// 温度控制
void IMU::TempControl::read_temp()
{
    static uint8_t tx_buf[4] = {0x22, 0, 0, 0};
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, temp_data, 4);
}

// 阻塞
void IMU::TempControl::read_temp_block()
{
    bmi088_accel_read_reg(0x22, temp_data, 2);
}

void IMU::TempControl::transform_data(uint8_t* data)
{
    int16_t temp_raw = ((uint16_t)data[0] << 3) | (data[1] >> 5);

    if (temp_raw & 0x0400)
        temp_raw -= 2048;

    temp_ = 23.0f + temp_raw * 0.125f;
}

void IMU::TempControl::handle()
{
    uint16_t pwm = static_cast<uint16_t>(std::fmax(std::round(pid_.calc(temp_ref_, temp_)), 0.0f));
    htim10.Instance->CCR1 = pwm;
}

bool IMU::TempControl::try_control()
{
    uint32_t now = HAL_GetTick();
    if (now - last_tick > 1300U)
    {
        read_temp();
        last_tick = now;
        return true;
    }
    return false;
}

void IMU::TempControl::temp_control(uint16_t time_ms)
{
    uint32_t tick = HAL_GetTick();
    uint32_t now;
    do
    {
        read_temp_block();
        transform_data(temp_data);
        handle();

        HAL_Delay(1300);
        now = HAL_GetTick();
    }
    while (now < tick + time_ms);
}
