//
// Created by Tessia on 2023/12/10.
//

#include "motor_driver.h"

void motor_dev::motor_set_rounds(float rounds) {
    if(reverse == 1){
        motor_set_rounds_forward(-rounds);
    } else{
        motor_set_rounds_forward(rounds);
    }
}

void motor_dev::motor_set_speed(float speed) {
    if(reverse == 1){
        motor_set_speed_forward(-speed);
    } else{
        motor_set_speed_forward(speed);
    }
}

void motor_dev::motor_set_current(float current) {
    if(reverse == 1){
        motor_set_current_forward(-current);
    } else{
        motor_set_current_forward(current);
    }
}

float motor_dev::motor_get_rounds() {
    if(reverse == 1){
        return(-motor_get_rounds_forward());
    } else{
        return(motor_get_rounds_forward());
    }
}

float motor_dev::motor_get_speed() {
    if(reverse == 1){
        return(-motor_get_speed_forward());
    } else{
        return(motor_get_speed_forward());
    }
}


/**--------------------------以下虚函数没有定义会报错-----------------------------**/
float motor_dev::motor_get_speed_forward(){
    return(0);
}

float motor_dev::motor_get_rounds_forward(){
    return(0);
}

void motor_dev::motor_set_current_forward(float current){
    ;
}

void motor_dev::motor_set_speed_forward(float current){
    ;
}

void motor_dev::motor_set_rounds_forward(float current){
    ;
}

bool motor_dev::motor_reset(){
    return(false);
}

void motor_dev::motor_control(uint32_t cmd){
    ;
}