/**
  ******************************************************************************
  * @file           : pwm_motor_driver.cpp
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/1/17
  ******************************************************************************
  */



#include "pwm_motor_driver.h"

pwm_motor::pwm_motor(TIM_HandleTypeDef *tim_, uint32_t tim_channel_)
          :tim(tim_),tim_channel(tim_channel_),pwmVal(0),motor_speed(0),duty_cycle(0.5){
    Prescaler = tim->Init.Prescaler;
    Period = tim->Init.Period;

    HAL_TIM_PWM_Start(tim, tim_channel);
    //校准需要
    modify_duty_cycle(duty_cycle);
    osDelay(1000); //todo：无法同时校准，有时间改改，现在问题不大
}


//占空比只能大于0.5
void pwm_motor::modify_duty_cycle(float duty_cycle_){
    pwmVal = (uint32_t)(duty_cycle_ * Period);
    __HAL_TIM_SetCompare(tim, tim_channel, pwmVal);
}

void pwm_motor::motor_set_speed_forward(float speed){
    speed = fabs(speed);
    motor_speed = speed;
    duty_cycle = 0.5 + speed * 0.5;
    modify_duty_cycle(duty_cycle);
}

float pwm_motor::motor_get_speed_forward(){
    return(motor_speed);
}



/****------------------------------不应调用---------------------------------***/

float pwm_motor::motor_get_rounds_forward(){
    return(0);
}

void pwm_motor::motor_set_current_forward(float current){
    ;
}

void pwm_motor::motor_set_rounds_forward(float current){
    ;
}

bool pwm_motor::motor_reset(){
    return(false);
}

void pwm_motor::motor_control(uint32_t cmd){
    ;
}