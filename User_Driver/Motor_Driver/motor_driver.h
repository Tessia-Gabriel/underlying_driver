//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_MOTOR_DRIVER_H


#include "cmsis_os.h"

class motor_dev {
protected:
    virtual float motor_get_speed_forward();  //返回电机转子速度 rpm 际实际值 不做归一化
    virtual float motor_get_rounds_forward(); //返回电机转子圈数 实际值 不做归一化

    virtual void motor_set_current_forward(float current);  //范围 -1～1
    virtual void motor_set_speed_forward(float speed);      //范围 -1～1
    virtual void motor_set_rounds_forward(float rounds);    //单位 转子圈数



public:
/***----------------------------------实际调用--------------------------------------**/
    float motor_get_speed();                        //返回电机转子圈数 实际值 不做归一化
    float motor_get_rounds();                       //返回电机转子圈数 实际值 不做归一化

    void motor_set_current(float current);  //范围 -1～1
    void motor_set_speed(float speed);      //范围 -1～1
    void motor_set_rounds(float rounds);    //单位 转子圈数

    virtual bool motor_reset();
    virtual void motor_control(uint32_t cmd);

protected:
    uint32_t reverse;    //是否反转
};



#endif //PEPPER_PICKER_GIMBAL_MOTOR_DRIVER_H
