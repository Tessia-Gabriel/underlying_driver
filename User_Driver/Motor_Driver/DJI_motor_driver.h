/**
  ******************************************************************************
  * @file           : DJI_motor_driver.h
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
  */

#ifndef PEPPER_PICKER_GIMBAL_DJI_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_DJI_MOTOR_DRIVER_H

//C

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "motor_driver.h"
#include "CAN_bsp.h"
#include "pid_driver.h"
#include "string.h"
#include "main.h"


const float MOTOR_6020_MAX_SPEED = 320.0f;
const float MOTOR_3508_MAX_SPEED = 9600.0f;
const float MOTOR_2006_MAX_SPEED = 21000.0f;
const float ENCODER_ANGLE_RATIO = 22.755555555555556f;     //(8192.0f / 360.0f)
const float ENCODER_TO_ANGLE_RATIO = 0.0439453125f;        // 360/8192.0
const float ENCODER_MAX = 8192.0f;
const float ENCODER_TO_ROUND = 1.0f/8192.0f;
const float MOTOR_6020_MAX_CURRENT = 16384;                //16384 for M3508
const float MOTOR_3508_MAX_CURRENT = 16384;                //16384 for M3508
const float MOTOR_2006_MAX_CURRENT = 10000;

enum DJI_MOTOR_TYPE {
    DJI_M3508 = 0, //默认为
    DJI_M2006 = 1,
    DJI_GM6020 = 2, ///推荐id为5、6、7
};
enum DJI_CMD {
    DJI_MOTOR_CLEAR_PID = 200,
    DJI_MOTOR_RESET_OFFSET,
};

struct dji_motor_data {
    uint16_t last_ecd;
    uint16_t ecd;           //转子机械角度 编码器值
    int16_t speed_rpm;      //转速
    int16_t given_current;  //实际电流

    int32_t round_cnt;      //圈数计算
    int32_t total_ecd;      //总机械角

    float total_round;      //总圈数
    int32_t ecd_raw_rate;   //差值
    uint32_t msg_cnt;       //计数用于
    uint16_t offset_ecd;    //校准偏置
};


class DJI_motor : public motor_dev {

public:
    /***--------------------------初始化时用------------------------------***/
    DJI_motor(CAN_HandleTypeDef *hcan_=&hcan1, can_rx_callback *callback_ = nullptr,
            uint32_t id_motor = 1, DJI_MOTOR_TYPE type_ = DJI_M3508, uint8_t is_reverse = 0);

    /***--------------------------指令------------------------------***/
    bool motor_reset() override;                             //复位
    void motor_control(uint32_t cmd) override;

protected:
    /****------------------------------电机运行时用---------------------------------***/
    float motor_get_speed_forward() override;  //返回电机转子速度 rpm 际实际值 不做归一化
    float motor_get_rounds_forward() override; //返回电机转子圈数 实际值 不做归一化

    void motor_set_current_forward(float current) override;  //范围 -1～1
    void motor_set_speed_forward(float speed) override;      //范围 -1～1
    void motor_set_rounds_forward(float rounds) override;    //单位 转子圈数


    /****------------------------------回执函数及其他---------------------------------***/
    friend void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data);
    void dji_motor_can_tx_write();

protected:
    enum DJI_MOTOR_TYPE type;
    uint32_t id;            //电机id
    dji_motor_data motor_data;  //电机数据
    bool init_offset_f;     //校准标志
    bool is_id_false;       //id是否发生错误
    int16_t current_to_send;        //要发送的电流
    can_device_receive can_rx;
    can_device_transmit can_tx;
    uint8_t bias_ordinal;
    DJI_CMD cmd;

public:
    uint8_t can_rx_data[8];
    uint8_t *can_tx_data;
    pid velPid;
    pid posPid;
};

#ifdef __cplusplus
extern "C" {
#endif

void DJI_motor_service(void *argument);

#ifdef __cplusplus
}
#endif
//C++

#endif //PEPPER_PICKER_GIMBAL_DJI_MOTOR_DRIVER_H
