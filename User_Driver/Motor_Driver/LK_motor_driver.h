//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_LK_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_LK_MOTOR_DRIVER_H


#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "motor_driver.h"
#include "CAN_bsp.h"
#include "pid_driver.h"
#include "string.h"
#include "main.h"

const float LK_MS_5010_MAX_SPEED_RPM = 505.0f;                                          //单位rpm，最大转速，转每分钟
const float LK_MS_5010_MAX_SPEED_DPS = LK_MS_5010_MAX_SPEED_RPM * 360.0f / 60.0f;       //单位dps，度每秒
const float LK_MS_5010_MAX_POWER = 1000.0f;                                             //返回值为1000~-1000，但实际最大功率为4.7W
const uint32_t LK_MS_5010_MAX_ECD = 65536;                                              //编码器最大值(达不到，要减1)
const float LK_MS_5010_ECD2ROUND = 1.0f / 65536.0f;                                     //编码器最大值(达不到，要减1)


const float LK_MS_6015_MAX_SPEED_RPM = 360.0f;                                          //单位rpm，最大转速，转每分钟
const float LK_MS_6015_MAX_SPEED_DPS = LK_MS_6015_MAX_SPEED_RPM * 360.0f / 60.0f;       //单位dps，度每秒
const float LK_MS_6015_MAX_POWER = 1000.0f;                                             //返回值为1000~-1000，但实际最大功率为9.8W
const uint32_t LK_MS_6015_MAX_ECD = 32768;                                              //编码器最大值(达不到，要减1)
const float LK_MS_6015_ECD2ROUND = 1.0f / 32768.0f;                                     //编码器最大值(达不到，要减1)

///多电机控制目前只支持转矩开环控制，一是没有写PID命令了，二是我懒得写了
enum LK_motor_mode{
    pos_ctrl_2 = 0,  //  位置闭环控制命令2（似乎要调三个环的参数，转矩环也有），电机内置pid
    speed_ctrl = 1,  //  速度闭环控制，电机内置pid
    torque_ctrl = 2  //  转矩开环控制，单片机内置pid
};

enum pid_mode{ //单纯为了好看，选择pid在单片机算还是电机算
    micro_pid = 0,
    motor_pid = 1
};

enum LK_motor_cmd{
    read_pi = 0,
    write_pi = 1,
    read_status = 2,
    LK_disable_offset = 3, //将偏置消除
    LK_add_one_lap_offset = 4,  //适用于初始化过零点问题，加一圈偏置
    LK_sub_one_lap_offset = 5,  //适用于初始化过零点问题，减一圈偏置
};

enum LK_motor_type{
    ms_5010 = 0,
    ms_6015 = 1,
};

struct LK_motor_data {
    uint16_t last_ecd;
    uint16_t ecd;           //转子机械角度 编码器值
    int16_t speed_rpm;      //转速
    int16_t power;          //输出功率
    int8_t temperature;     //温度

    int32_t round_cnt;      //圈数计算
    int32_t total_ecd;      //总机械角

    float total_round;      //总圈数
    int32_t ecd_raw_rate;   //差值
    uint32_t msg_cnt;       //计数用于
    int64_t offset_ecd;    //校准偏置
};

struct LK_motor_param{
    float max_speed_dps;
    float max_power;
    float max_ecd;  //编码器最大值，随18bit，14bit自行定义
    float ecd2round;
};


/**
 *
 * 实际调用函数应为从父类继承的函数，这里没有定义和声明
 */
class LK_motor : public motor_dev{
public:
    /***--------------------------初始化时用------------------------------***/
    LK_motor(CAN_HandleTypeDef *hcan_=&hcan1, can_rx_callback *callback_ = nullptr, uint8_t is_use_multi_ctrl_ = 0,
             uint32_t id_motor = 1, LK_motor_type motor_type = ms_5010, uint8_t is_reverse = 0, uint8_t is_use_motor_pid_ = 0);

    void motor_control(uint32_t cmd) override;
    bool motor_reset() override;                             //复位
    bool motor_pid_send(); ///这个函数只适合pos_ctl_2用
    void motor_set_offset(uint32_t offset); //未归一化


protected:
    /****------------------------------电机运行时用---------------------------------***/
    float motor_get_speed_forward() override;  //返回电机转子速度 rpm 际实际值 不做归一化
    float motor_get_rounds_forward() override; //返回电机转子圈数 实际值 不做归一化

    void motor_set_current_forward(float current) override;  //范围 -1～1
    void motor_set_speed_forward(float speed) override;      //范围 -1～1
    void motor_set_rounds_forward(float rounds) override;    //单位 转子圈数


    /****------------------------------回执函数---------------------------------***/
    friend void LK_motor_update_data_callback(can_device_receive *can_receive, uint8_t *data);

protected:
    can_device_receive can_rx;
    can_device_transmit can_tx;
    LK_motor_mode mode;
    LK_motor_type type;
    bool is_pid_send_success;
    bool is_init_success; //需手动置1，置1后CAN发送自动开启
    LK_motor_data motor_data;
    int16_t power_send; //要发送的功率
    uint8_t is_use_motor_pid; //是否使用电机内置pid，如果是要给三个环的参数
    uint8_t motor_id;
    pid_mode pidMode;
    LK_motor_param motor_param;
    uint8_t is_use_multi_ctrl;

public:
    uint8_t default_data_rx[8];
    uint8_t *default_data_tx;
    pid posPid;
    pid velPid;
    pid torPid; //只有电机内置pid的时候会用，且只会用到pi参数
};


extern "C" {
void LK_motor_service(void *argument);
}



#endif //PEPPER_PICKER_GIMBAL_LK_MOTOR_DRIVER_H
