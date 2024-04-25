/**
  ******************************************************************************
  * @file           : can_test_task.h
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/25
  ******************************************************************************
  */



#ifndef UNDERLYING_DRIVER_CAN_TEST_TASK_H
#define UNDERLYING_DRIVER_CAN_TEST_TASK_H
#ifdef __cplusplus

#include "CAN_bsp.h"

typedef struct {        // 单位
    float V_IN;         // V
    float V_SC;         // V
    float I_IN_SC;      // A
    float I_CHASSIS;    // A
}SC_feedback_t;

typedef struct {
    uint8_t enable;
    float set_power_charge;     //发送给电容控制板数据，给电容充电为正，放电为负（单位 W）
}SC_set_t;

typedef struct {
    SC_feedback_t scFeedback;
    SC_set_t scSet;
    struct {
        float p_sc;			//电容发送回来的电容功率数据  充电为正
        float p_wheel;		//四轮电机功率
        float last_p_wheel;
    } power_fb;
} Super_Cap_t;

enum SuperCapState{    //电容充放电情况	0 电容没电	1 电容可用	2 电容满电
    FULL = 0,
    READY = 1,
    EMPTY = 2,
    SUPERC_ERROR = 3,
    SUPERC_DEBUG = 4
};

#pragma pack(1)
typedef struct {
    float chassis_yaw; //单位deg
    uint8_t is_target_detected: 1;
    uint8_t is_chassis_yaw_stop: 1;
    uint8_t is_spin: 1;
    uint8_t is_use_capacitor: 1;
    uint64_t reserve: 28;
} chassis_yaw_info_tran;
#pragma pack()

typedef struct {
    float chassis_yaw; //单位deg
    uint8_t is_target_detected;
    uint8_t is_chassis_yaw_stop;
    uint8_t is_spin;
    uint8_t is_use_capacitor;
} chassis_yaw_info;

#pragma pack(1)
typedef struct {
    float yaw_angle;
    uint8_t is_flip_over: 1;
    uint8_t reverse_0: 7;
    uint8_t reverse_1: 8;
    uint8_t reverse_2: 8;
    uint8_t reverse_3: 8;
} chassis_info_tran;
#pragma pack()


extern "C" {
#endif
//C
void can_test_task(void *argument);
#ifdef __cplusplus
}
#endif
//C++

#endif //UNDERLYING_DRIVER_CAN_TEST_TASK_H
