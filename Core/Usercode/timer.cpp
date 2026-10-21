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
        static uint32_t last_tick = 0;
        uint32_t now = HAL_GetTick();

        if (last_tick == 0)
        {
            last_tick = now;
            return;
        }

        uint32_t elapsed = now - last_tick;
        last_tick = now;

        if (elapsed >= 1U)
        {
            last_tick = now;

            // 实际时间间隔，单位为秒
            float dt_s = elapsed * 0.001f;

            imu.update(dt_s);
        }
    }
}

extern "C" {
void imu_init()
{
    imu.acc_init();
    imu.gyro_init();

    imu.temp_control_.temp_control(500);

    imu.gyro_calibrate(2000);
}
}