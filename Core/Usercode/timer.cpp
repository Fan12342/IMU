//
// Created by Administrator on 2026/10/18.
//

#include "timer.h"
#include "bmi088.h"
#include "IMU.h"
#include "stm32f4xx_hal_spi.h"

uint8_t acc_id;
uint8_t gyro_id;
IMU imu;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        static uint32_t last_tick = 0;
        uint32_t now = HAL_GetTick();

        if (last_tick == 0)
        {
            last_tick = now;
            return;
        }

        uint32_t dt_ms = now - last_tick;
        last_tick = now;
        imu.update(dt_ms);
    }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        imu.dmaCallback();
    }
}

extern "C" {
void imu_init()
{
    imu.acc_init();
    imu.gyro_init();

    //imu.temp_control_.temp_control(500);

    imu.gyro_calibrate(1000);
}
}