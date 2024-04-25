/**
  ******************************************************************************
  * @file           : can_test_task.cpp
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/25
  ******************************************************************************
  */



#include "can_test_task.h"

Super_Cap_t superCap;
SuperCapState super_cap_state;

chassis_yaw_info chassis_yaw_cal;

chassis_info_tran chassis_info;

float yaw_gyro;
uint8_t is_flip_over;


void can_super_capacitor_callback(can_device_receive *can_receive, uint8_t *data) {
    float V_SC;
    superCap.power_fb.p_sc = (int16_t)(data[0] | (data[1] << 8)) / 50.f;           //0.02W/LSB
    superCap.power_fb.p_wheel = (int16_t)(data[2] | (data[3] << 8)) / 50.f;        //0.02W/LSB
    V_SC = (int16_t)(data[4] | (data[5] << 8)) / 1000.f;                           //mV -> V
    superCap.scFeedback.V_SC = superCap.scFeedback.V_SC * 0.9f + V_SC * 0.1f;      //低通
    super_cap_state = (SuperCapState)data[6];
}

void can_chassis_yaw_cal_callback(can_device_receive *can_receive, uint8_t *data){
    chassis_yaw_info_tran *chassis_yaw_cal_tran = (chassis_yaw_info_tran *)data;

    chassis_yaw_cal.chassis_yaw = chassis_yaw_cal_tran->chassis_yaw;
    chassis_yaw_cal.is_target_detected = chassis_yaw_cal_tran->is_target_detected;
    chassis_yaw_cal.is_chassis_yaw_stop = chassis_yaw_cal_tran->is_chassis_yaw_stop;
    chassis_yaw_cal.is_spin = chassis_yaw_cal_tran->is_spin;
    chassis_yaw_cal.is_use_capacitor = chassis_yaw_cal_tran->is_use_capacitor;
}



void can_test_task(void *argument){
    /**
     * CAN接收只需在任务中如下定义一个变量，并自定义接收回执函数即可
     * 如下，这样便能接收到对应帧头的can消息了
     */
    can_device_receive can_super_capacitor_receive(&hcan2, 0x311, can_super_capacitor_callback);
    can_device_receive can_chassis_yaw_cal(&hcan2, 0x305, can_chassis_yaw_cal_callback);

    /**
    * CAN发送需任务中如下定义一个变量，并定义一个最大8字节的结构体或数组，在for循环中不断更新并发送
    */
    can_device_transmit can_chassis_info(&hcan2, 0x301, 8, (uint8_t *)&chassis_info);

    for(;;){
        taskENTER_CRITICAL();
        chassis_info.yaw_angle = yaw_gyro;
        chassis_info.is_flip_over = is_flip_over;

        taskEXIT_CRITICAL();
        can_chassis_info.can_send_msg();   //这里是发送队列中，用户不用担心can阻塞问题
        osDelay(1);
    }
}