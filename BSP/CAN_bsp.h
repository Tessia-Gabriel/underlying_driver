//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_CAN_BSP_H
#define PEPPER_PICKER_GIMBAL_CAN_BSP_H


#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "can.h"

#include "string.h"


class can_device_receive;

typedef void (can_rx_callback)(can_device_receive *can_receive, uint8_t *data);


struct can_device_transmit_member{
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint8_t len;
    uint8_t *buf_data;
};


/**
 *
 * 关于callback导致类中指针无法转化为普通指针的问题解决办法
 * 1.删除callback，使用信号量或其他（下策，太呆了，每次建一个can_rx都得建一个信号量）
 * 2.用友元函数建立类中的callback，再使用锦程的container_of找到上层
 *
 */
//想来想去还是函数指针合适
class can_device_receive {

public:
    can_device_receive(CAN_HandleTypeDef *hcan_=&hcan1, uint32_t id_=0, can_rx_callback *callback_ = nullptr);
    bool can_set_callback(can_rx_callback *callback_);
    void can_modify_id(uint32_t id_);

private:
    CAN_HandleTypeDef *hcan;    //通过函数传递的参数
    uint32_t id;        //通过函数传递的参数
    uint32_t index;     //计算出的索引


public:
    uint8_t rx_data[8]; //原始数据
    can_rx_callback *callback;    //需要直接赋值
    uint8_t is_callback_none;     //未定义回执函数
};



class can_device_transmit {

public:
    can_device_transmit(CAN_HandleTypeDef *hcan_, uint32_t id_ = 0, uint8_t len_ = 8, uint8_t *buf_data_ = nullptr); //init
    can_device_transmit();
    osStatus_t can_send_msg(); //发送
    bool set_buf_address(uint8_t *buf_address);
    bool set_id(uint32_t id);

public:
    can_device_transmit_member member;
};




#ifdef __cplusplus
extern "C" {
#endif

void can1_sending_service(void *argument);
void can2_sending_service(void *argument);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);


#ifdef __cplusplus
} // extern "C"
#endif





#endif //PEPPER_PICKER_GIMBAL_CAN_BSP_H
