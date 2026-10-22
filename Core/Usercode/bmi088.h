//
// Created by Administrator on 2026/10/17.
//

#ifndef IMA_BMI088_H
#define IMA_BMI088_H
#include "../Inc/spi.h"
#include "stm32f4xx_hal_def.h"


void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t* rx_data, uint8_t length);
void bmi088_write_reg(uint8_t reg, uint8_t data);

void BMI088_ACCEL_NS_L();
void BMI088_ACCEL_NS_H();
void BMI088_GYRO_NS_L();
void BMI088_GYRO_NS_H();

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t tx_data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length); // 加速度计读取，注意需要忽略第一个字节dummy byte
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length); // 陀螺仪读取
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data); // gyro写入

#ifdef __cplusplus
extern "C" {
#endif

uint8_t bmi088_init();

#ifdef __cplusplus
}
#endif
#endif //IMA_BMI088_H
