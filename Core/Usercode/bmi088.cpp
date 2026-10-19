//
// Created by Administrator on 2026/10/17.
//

#include "bmi088.h"

// ✔
void bmi088_write_byte(uint8_t tx_data)
{
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
}

// ✔
void bmi088_read_byte(uint8_t* rx_data, uint8_t length)
{
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}

// ✔
void bmi088_write_reg(uint8_t reg, uint8_t data)
{
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);
}



// ✔
void BMI088_ACCEL_NS_L()
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

// ✔
void BMI088_ACCEL_NS_H()
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

// ✔
void BMI088_GYRO_NS_L()
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

// ✔
void BMI088_GYRO_NS_H()
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}



// ✔
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t tx_data) {
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(tx_data);

    BMI088_ACCEL_NS_H();
}

// ✔
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(tx_data);

    BMI088_GYRO_NS_H();
}

// ✔
// 加速度计读取，注意需要忽略第一个字节dummy byte
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length)
{
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(rx_data, 1);
    bmi088_read_byte(rx_data, length);

    BMI088_ACCEL_NS_H();
}

// ✔
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(rx_data, length);

    BMI088_GYRO_NS_H();
}



// ✔
extern "C" {
uint8_t bmi088_init() {
    uint8_t dummy = 0;
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    // uint8_t acc_power[2] = {0};
    // uint8_t acc_config[2] = {0};
    // uint8_t gyro_config[3] = {0};

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    HAL_Delay(1);

    // Switch ACCEL to SPI，第一次读到的值不用
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Soft Reset ACCEL
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(1);

    // Soft Reset GYRO
    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(30);

    // ACCEL 软复位后再做一次虚拟读取
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Check Chip ID
    bmi088_accel_read_reg(0x00, &acc_id, 1);
    if (acc_id != 0x1E) {
        return 0;
    }
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) {
        return 0;
    }

    // Switch ACCEL to Active Mode
    bmi088_accel_write_single_reg(0x7C, 0x00); // ACC_PWR_CONF
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7D, 0x04); // ACC_PWR_CTRL
    HAL_Delay(5);

    // ACCEL Range / ODR / BW
    bmi088_accel_write_single_reg(0x41, 0x01); // ACC_RANGE
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x40, 0xAB); // ACC_CONF
    HAL_Delay(40);

    // GYRO Power / Range / ODR / BW
    bmi088_gyro_write_single_reg(0x11, 0x00); // GYRO_LPM1
    HAL_Delay(30);
    bmi088_gyro_write_single_reg(0x0F, 0x00); // GYRO_RANGE
    return 1;
}
}