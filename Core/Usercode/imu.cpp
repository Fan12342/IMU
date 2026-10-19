//
// Created by Administrator on 2026/10/18.
//

#include "imu.h"

#include <ctgmath>
#include "bmi088.h"

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
    acc_data_to_mg_ = 1000.0f * 1.5f * pow(2, raw_range + 1) / 32768.0f;
}

void IMU::gyro_init()
{
    uint8_t raw_range;
    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    bmi088_accel_read_reg(0x0F, &raw_range, 1);
    gyro_data_to_dps_ = 2000.0f / pow(2, raw_range & 0x07) / 32768.0f;
}

void IMU::sensorDataTransform()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro_sensor_[i] = raw_data_.gyro[0] * R_imu_[i][0] + raw_data_.gyro[1] * R_imu_[i][1] + raw_data_.gyro[2] *
            R_imu_[i][2] - gyro_bias_[i];
        accel_sensor_[i] = accel_sen_ * (raw_data_.accel[0] * R_imu_[i][0] + raw_data_.accel[1] * R_imu_[i][1] +
            raw_data_.accel[2] * R_imu_[i][2]) - accel_bias_[i];
    }
}

void IMU::update()
{
    acc_calculate();
    gyro_calculate();

    sensorDataTransform();
}
