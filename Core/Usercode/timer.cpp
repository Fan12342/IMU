//
// Created by Administrator on 2026/10/18.
//

#include "timer.h"
#include "bmi088.h"
#include "imu.h"

uint8_t acc_id;
uint8_t gyro_id;
IMU imu;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        imu.update();
    }
}

extern "C" {
void imu_init()
{
    imu.acc_init();
    imu.gyro_init();

    imu.temp_control_.temp_control(500);

    imu.gyro_calibrate(500);
}
}