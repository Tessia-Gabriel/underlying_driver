/**
  ******************************************************************************
  * @file           : pwm_motor_driver.h
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/1/17
  ******************************************************************************
  */



#ifndef PEPPER_PICKER_GIMBAL_PWM_MOTOR_DRIVER_H
#define PEPPER_PICKER_GIMBAL_PWM_MOTOR_DRIVER_H

#include "math.h"
#include "tim.h"
#include "motor_driver.h"

class pwm_motor : public motor_dev{

public:
    pwm_motor(TIM_HandleTypeDef *tim_, uint32_t tim_channel_);

protected:
    /****------------------------------电机运行时用---------------------------------***/
    float motor_get_speed_forward() override;                //只是返回设定值而已
    void motor_set_speed_forward(float speed) override;      ///范围 0～1，只用这个，起码snail是这样

    void modify_duty_cycle(float duty_cycle); //参数应为0-1之间

    /****------------------------------不应调用---------------------------------***/
    float motor_get_rounds_forward() override;
    void motor_set_current_forward(float current) override;
    void motor_set_rounds_forward(float rounds) override;
    void motor_control(uint32_t cmd) override;
    bool motor_reset() override;


protected:
    TIM_HandleTypeDef *tim;
    uint32_t tim_channel;
    uint32_t Prescaler;     //分频系数
    uint32_t Period;        //计数值
    uint32_t pwmVal;        //向上计数触发值
    float duty_cycle;       //占空比，只能大于0.5
    float motor_speed;      //范围0-1
};





#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++

#endif //PEPPER_PICKER_GIMBAL_PWM_MOTOR_DRIVER_H
