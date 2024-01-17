//
// Created by DELL on 2023/12/11.
//

#include "motor_test_task.h"

void motor_test_task(void *argument){
    DJI_motor dji_motor_test(&hcan1, nullptr, 1, DJI_M3508, 1);
//    LK_motor lk_motor_test(&hcan1,nullptr, 1, ms_6015);
//    DM_motor dm_motor_test(&hcan1,nullptr,0x000,0x001);
//    pwm_motor pwm_motor_test(&htim1,TIM_CHANNEL_1);

    for(;;){
        dji_motor_test.motor_set_speed(-0.1);
//        dji_motor_test.motor_set_rounds(0.5);
//        lk_motor_test.motor_set_speed(0.2);
//        lk_motor_test.motor_set_rounds(0);
//        dm_motor_test.motor_set_speed(0.1);
//        dm_motor_test.motor_set_rounds(0);
//        pwm_motor_test.motor_set_speed(0.1);
        osDelay(1);
    }
}
