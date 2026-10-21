//
// Created by Administrator on 2026/10/18.
//

#include "imu.h"

#include <ctgmath>
#include "bmi088.h"
#include "tim.h"

void IMU::acc_calculate()
{
    // 2. 读取acc0x12寄存器中的6位acc数据
    uint8_t rx_acc_data[6];
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    // 3. 用量程系数将原始数据转换为常用单位
    int16_t Accel_X = (rx_acc_data[1] << 8) | rx_acc_data[0];

    int16_t Accel_Y = (rx_acc_data[3] << 8) | rx_acc_data[2];

    int16_t Accel_Z = (rx_acc_data[5] << 8) | rx_acc_data[4];

    raw_data_.accel[0] = static_cast<float>(Accel_X) * acc_data_to_mg_;
    raw_data_.accel[1] = static_cast<float>(Accel_Y) * acc_data_to_mg_;
    raw_data_.accel[2] = static_cast<float>(Accel_Z) * acc_data_to_mg_;
}

void IMU::gyro_calculate()
{
    // 2. 读取gyro0x02寄存器中的6位gyro数据
    uint8_t rx_gyro_data[6];
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);

    // 3. 用量程系数将原始数据转换为常用单位
    int16_t Rate_X = (rx_gyro_data[1] << 8) | rx_gyro_data[0];

    int16_t Rate_Y = (rx_gyro_data[3] << 8) | rx_gyro_data[2];

    int16_t Rate_Z = (rx_gyro_data[5] << 8) | rx_gyro_data[4];

    raw_data_.gyro[0] = static_cast<float>(Rate_X) * gyro_data_to_dps_;
    raw_data_.gyro[1] = static_cast<float>(Rate_Y) * gyro_data_to_dps_;
    raw_data_.gyro[2] = static_cast<float>(Rate_Z) * gyro_data_to_dps_;
}

void IMU::acc_init()
{
    uint8_t raw_range;
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    acc_data_to_mg_ = 1000.0f * 3.0f * static_cast<float>(1U << (raw_range & 0x03)) / 32768.0f;
}

void IMU::gyro_init()
{
    uint8_t raw_range;
    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    gyro_data_to_dps_ = 2000.0f / static_cast<float>(1U << (raw_range & 0x07)) / 32768.0f;
}

void IMU::sensorDataTransform()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro_sensor_[i] = -gyro_bias_[i];
        for (uint8_t j = 0; j < 3; j++)
            gyro_sensor_[i] += R_imu_[i][j] * raw_data_.gyro[j];

        accel_sensor_[i] = -accel_bias_[i];
        for (uint8_t j = 0; j < 3; j++)
            accel_sensor_[i] += R_imu_[i][j] * raw_data_.accel[j];
    }
}

void IMU::update()
{
    acc_calculate();
    gyro_calculate();

    sensorDataTransform();


    static uint32_t last_temp_tick = 0;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_temp_tick) >= 1300U)
    {
        last_temp_tick = now;
        temp_control_.read_temperature();
    }
}

void IMU::gyro_calibrate(uint32_t time_ms)
{
    float raw_data_sum[3] = {0.0f, 0.0f, 0.0f};

    uint32_t samples = 0;

    // 求和
    uint32_t start_tick = HAL_GetTick();
    do
    {
        samples++;
        gyro_calculate();
        for (uint8_t i = 0; i < 3; i++)
            raw_data_sum[i] += raw_data_.gyro[i];
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

void IMU::TempControl::read_temperature()
{
    uint8_t temp_data[2];

    bmi088_accel_read_reg(0x22, temp_data, 2);

    // 拼接 11 位温度数据
    int16_t temp_raw = ((uint16_t)temp_data[0] << 3) | (temp_data[1] >> 5);

    // 转换为 11 位有符号数
    if (temp_raw & 0x0400)
        temp_raw -= 2048;

    temperature_ = 23.0f + temp_raw * 0.125f;
}

void IMU::TempControl::temp_control(uint16_t time_ms)
{
    uint32_t tick = HAL_GetTick();
    uint32_t now;
    do
    {
        handle();
        HAL_Delay(1);
        now = HAL_GetTick();
    }
    while (now < tick + time_ms);
}

void IMU::TempControl::handle()
{
    read_temperature();
    uint16_t pwm = static_cast<uint16_t>(std::max(std::round(pid_.calc(temp_ref_, temperature_)), 0.0f));
    output_ = pwm;
    htim10.Instance->CCR1 = pwm;
}
