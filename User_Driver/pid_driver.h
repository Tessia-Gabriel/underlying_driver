//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_PID_DRIVER_H
#define PEPPER_PICKER_GIMBAL_PID_DRIVER_H



#include "stdbool.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"



struct pid_param {
    float p;
    float i;
    float d;
    float input_max_err;//不用
    float max_out;
    float integral_higher_limit; //积分限幅 正
    float integral_lower_limit;  //积分限幅 负
};


class pid{

public:
    pid(float max_out_ = 1.0,
        float integral_limit = 0.1,

        float kp = 5.0,
        float ki = 0.001,
        float kd = 0.5);

    float pid_calculate(float set_, float get_);
    void pid_reset(float max_out, float integral_limit, float kp, float ki, float kd);
    void pid_clear();

protected:

    bool enable;

    float set;
    float get;

    float err;
    float last_err;
    float penultimate_err;

    float pout;
    float iout;
    float dout;


public:
    pid_param param;
    float out;

};

#endif //PEPPER_PICKER_GIMBAL_PID_DRIVER_H
