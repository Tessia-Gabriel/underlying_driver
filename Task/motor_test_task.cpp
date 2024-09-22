//
// Created by DELL on 2023/12/11.
//

#include "motor_test_task.h"
float rounds_tmp = 0;

void motor_test_task(void *argument){
//    DJI_motor dji_motor_test(&hcan1, nullptr, 6, DJI_M2006, 1);
//    LK_motor lk_motor_test(&hcan1,nullptr, 1, ms_6015);
//    DM_motor dm_motor_test(&hcan1,nullptr,0x000,0x001);
    static HT_motor ht_motor_test(&hcan1,nullptr,0x000,0x001,HT_8115_J9,1);
//    pwm_motor pwm_motor_test(&htim1,TIM_CHANNEL_1);

    for(;;){
//        dji_motor_test.motor_set_rounds(-0.1);
//        dji_motor_test.motor_set_current(0);
//        lk_motor_test.motor_set_speed(0.2);
//        lk_motor_test.motor_set_rounds(0);
//        dm_motor_test.motor_set_speed(0.1);
//        dm_motor_test.motor_set_rounds(0);
//        pwm_motor_test.motor_set_speed(0.1);
//        ht_motor_test.motor_set_current(0);
//        ht_motor_test.motor_set_speed(0.05);
        ht_motor_test.motor_set_rounds(rounds_tmp);
        osDelay(1);
    }
}
