//
// Created by DELL on 2023/12/11.
//

#ifndef PEPPER_PICKER_GIMBAL_MOTOR_TEST_TASK_H
#define PEPPER_PICKER_GIMBAL_MOTOR_TEST_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "LK_motor_driver.h"
#include "DJI_motor_driver.h"
#include "DM_motor_driver.h"
#include "pwm_motor_driver.h"
#include "HT_motor_driver.h"

void motor_test_task(void *argument);


#ifdef __cplusplus
}
#endif

#endif //PEPPER_PICKER_GIMBAL_MOTOR_TEST_TASK_H
