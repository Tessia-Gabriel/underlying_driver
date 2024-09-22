/**
  ******************************************************************************
  * @file           : HT_motor_driver.h
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/9/20
  ******************************************************************************
  */



#ifndef UNDERLYING_DRIVER_HT_MOTOR_DRIVER_H
#define UNDERLYING_DRIVER_HT_MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "motor_driver.h"
#include "CAN_bsp.h"
#include "pid_driver.h"
#include "DM_motor_driver.h"


const float HT_8115_J9_P_MIN = -95.5f;    // Radians
const float HT_8115_J9_P_MAX = 95.5f;
const float HT_8115_J9_V_MIN = -45.0f;    // Rad/s
const float HT_8115_J9_V_MAX = 45.0f;
const float HT_8115_J9_KP_MIN = 0;     // N-m/rad
const float HT_8115_J9_KP_MAX = 500;
const float HT_8115_J9_KD_MIN = 0;     // N-m/rad/s
const float HT_8115_J9_KD_MAX = 5;
const float HT_8115_J9_T_MIN = -18;
const float HT_8115_J9_T_MAX = 18;

enum ht_motor_type{
    HT_8115_J9 = 0, //默认为
};

typedef struct {
    uint8_t id;
    uint16_t pos;       //转子机械角度 编码器值
    uint16_t vel;       //转速以rad/s为单位
    uint16_t current;   //电流
} ht_motor_raw_data;


#pragma pack(1)
typedef struct{
    uint16_t p_des : 16;
    uint16_t v_des : 12;
    uint16_t kp : 12;
    uint16_t kd : 12;
    uint16_t t_ff : 12;
} ht_motor_can_tx_buff;
#pragma pack()

typedef struct{
    float p_des; //位置给定
    float v_des; //速度给定
    float kp;
    float kd;
    float torq;
} ht_motor_tx;

typedef struct{
    uint16_t last_ecd;
    uint16_t ecd;           //转子机械角度 编码器值
    float current_round;    //浮点数当前位置  // 0-1
    float last_round;       //浮点数上次位置  // 0-1
    float delta_rounds;
    int16_t speed_rpm;      //转速
    float current_speed;    //浮点数当前速度  //-1-1
    float last_speed;       //浮点数上次速度  //-1-1
    int16_t given_current;  //实际电流
    float torque;           //力矩

    int32_t round_cnt;      //圈数计算
    int32_t total_ecd;      //总机械角

    float total_rounds;      //总圈数
    int32_t ecd_raw_rate;   //差值
    uint32_t msg_cnt;       //计数用于
    uint16_t offset_ecd;    //校准偏置
    float offset_round;     //浮点偏置
}ht_motor_data;


class HT_motor : public motor_dev {

public:
    /***--------------------------初始化------------------------------***/
    HT_motor(CAN_HandleTypeDef *hcan_=&hcan1, can_rx_callback *callback_ = nullptr,
             uint32_t rx_id = 0x000, uint32_t tx_id = 0x001, ht_motor_type type_ = HT_8115_J9, uint8_t is_reverse = 0);

    /***--------------------------指令------------------------------***/
    bool motor_reset() override;                             //复位
    void motor_control(uint32_t cmd) override;
    float motor_get_current_rounds();   //获取当前绝对值编码器值，即不加圈数


protected:
    /****------------------------------电机运行时用---------------------------------***/
    float motor_get_speed_forward() override;  //返回电机转子速度 rpm 际实际值 不做归一化
    float motor_get_rounds_forward() override; //返回电机转子圈数 实际值 不做归一化

    void motor_set_current_forward(float current) override;  //范围 -1～1
    void motor_set_speed_forward(float speed) override;      //范围 -1～1
    void motor_set_rounds_forward(float rounds) override;    //单位 转子圈数

    /****------------------------------回执函数及其他---------------------------------***/
    friend void ht_motor_rx_data_update_callback(can_device_receive *can_receive, uint8_t *data);
    void set_ht_ctrl_to_can_tx_buff();

protected:
    //basic
    bool is_lost;               //保护,初始化
    bool init_offset;        //校准标志,初始化
    ht_motor_type type;         //电机类型
    can_device_receive can_rx;
    can_device_transmit can_tx;
    uint8_t *default_data_tx;
    CAN_HandleTypeDef *hcan;
    uint8_t motor_num;        //该电机是第几个，从1计数，非0

    //data
    ht_motor_raw_data raw;    //原始数据,接收不用初始化
    ht_motor_data data;          //电机解算数据,接收不用初始化
    ht_motor_tx ctrl;
    ht_motor_can_tx_buff tx_buff;


public:
    pid velPid;
    pid posPid;
};




#ifdef __cplusplus
extern "C" {
#endif
//C
void HT_motor_service(void *argument);
#ifdef __cplusplus
}
#endif
//C++

#endif //UNDERLYING_DRIVER_HT_MOTOR_DRIVER_H
