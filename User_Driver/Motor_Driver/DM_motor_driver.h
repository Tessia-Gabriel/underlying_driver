/**
  ******************************************************************************
  * @file           : DM_motor_driver.h
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/1/16
  ******************************************************************************
  */



#ifndef PEPPER_PICKER_GIMBAL_DM_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_DM_MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "motor_driver.h"
#include "CAN_bsp.h"
#include "pid_driver.h"

const float RAD2ROUND = 1.0f/(2*PI);
const float DM_J4310_2EC_V_MAX = 30.0f;
const float DM_J4310_2EC_P_MAX = 12.5f;
const float DM_J4310_2EC_T_MAX = 10.0f;

const float DM_J4310_2EC_KP_MIN = 0.0f;
const float DM_J4310_2EC_KP_MAX = 500.0f;
const float DM_J4310_2EC_KD_MIN = 0.0f;
const float DM_J4310_2EC_KD_MAX = 5.0f;

//MIT做位控的话速度给0
enum dm_motor_mode{
    DM_MIT = 0,  //MIT模式
    DM_PV = 1,   //位置模式
    DM_VO = 2,   //速度模式
};

enum motor_type{
    DM_J4310_2EC = 0, //默认为
};

enum dm_cmd {
    enable_motor = 0,
    disable_motor = 1,
    save_zero_offset = 2,
    clear_pid = 3,
    DM_disable_offset = 4, //将偏置消除，如果绝对值编码器好用的话
};

typedef struct {
    uint8_t id;
    uint8_t err;
    uint16_t pos;       //转子机械角度 编码器值
    uint16_t vel;        //转速以rad/s为单位
    uint16_t torque;    //转矩
    uint8_t t_mos;      //驱动上平均温度℃
    uint8_t t_rotor;    //电机线圈温度℃
} dm_motor_raw_data;

#pragma pack(1)
typedef struct{
    uint16_t p_des : 16;
    uint16_t v_des : 12;
    uint16_t kp : 12;
    uint16_t kd : 12;
    uint16_t t_ff : 12;
} dm_motor_can_tx_buff;
#pragma pack()

typedef struct{
    float p_des; //位置给定
    float v_des; //速度给定
    float kp;
    float kd;
    float torq;
} dm_motor_tx;

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
}dm_motor_data;


class DM_motor : public motor_dev {

public:
    /***--------------------------初始化------------------------------***/
    DM_motor(CAN_HandleTypeDef *hcan_=&hcan1, can_rx_callback *callback_ = nullptr,
             uint32_t rx_id = 0x000, uint32_t tx_id = 0x001, motor_type type_ = DM_J4310_2EC, dm_motor_mode mode_ = DM_MIT, uint8_t is_reverse = 0);

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
    friend void dm_motor_rx_data_update_callback(can_device_receive *can_receive, uint8_t *data);
    void set_dm_ctrl_to_can_tx_buff();

protected:
    //basic
    bool is_lost;               //保护,初始化
    bool init_offset;        //校准标志,初始化
    motor_type type;         //电机类型
    dm_motor_mode mode;       //电机模式
    can_device_receive can_rx;
    can_device_transmit can_tx;
    uint8_t *default_data_tx;
    uint8_t motor_num;        //该电机是第几个，从1计数，非0
    CAN_HandleTypeDef *hcan;


    //data
    dm_motor_raw_data raw;    //原始数据,接收不用初始化
    dm_motor_data data;          //电机解算数据,接收不用初始化
    dm_motor_tx ctrl;
    dm_motor_can_tx_buff tx_buff;


public:
    pid velPid;
    pid posPid;
};


#ifdef __cplusplus
extern "C" {
#endif
//C
void DM_motor_service(void *argument);

#ifdef __cplusplus
}
#endif
//C++

#endif //PEPPER_PICKER_GIMBAL_DM_MOTOR_DRIVER_H
