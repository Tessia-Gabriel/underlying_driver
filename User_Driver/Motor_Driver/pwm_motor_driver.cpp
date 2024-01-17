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
          :tim(tim_),tim_channel(tim_channel_),pwmVal(0),motor_speed(0){
    Prescaler = tim->Init.Prescaler;
    Period = tim->Init.Period;

    HAL_TIM_PWM_Start(tim, tim_channel);
    motor_set_speed_forward(0);
}


void pwm_motor::modify_duty_cycle(float duty_cycle){
    pwmVal = (uint32_t)(duty_cycle * Period);
    __HAL_TIM_SetCompare(tim, tim_channel, pwmVal);
}

void pwm_motor::motor_set_speed_forward(float speed){
    speed = fabs(speed);
    motor_speed = speed;
    modify_duty_cycle(speed);
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