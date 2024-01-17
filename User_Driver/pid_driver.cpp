//
// Created by Tessia on 2023/12/10.
//

#include "pid_driver.h"

//绝对值限幅，两侧限幅相同
void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    }
    if (*a < -abs_max) {
        *a = -abs_max;
    }
}

//常数限幅，两侧限幅不同，这个是改变原值，但是也不返回值，返回值也可以的
void val_limit(float *a, float val_max,float val_min) {
    if (*a > val_max) {
        *a = val_max;
    }
    if (*a < val_min) {
        *a = val_min;
    }
}


pid::pid(float max_out_, float integral_limit, float kp, float ki, float kd)
    :param{kp, ki, kd, 0, max_out_, integral_limit, -integral_limit},//结构体初始化格式
     enable(true),
     set(0),get(0),
     err(0),last_err(0),penultimate_err(0),
     pout(0),iout(0),dout(0),out(0){ }

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output   0-outmax
  */ //0-outmax已限幅
__RAM_FUNC
float pid::pid_calculate(float set_, float get_) {
    get = get_;
    set = set_;
    err = set - get;
    if (fabsf(param.input_max_err) > 1e-6)
        abs_limit(&err, param.input_max_err);
    pout = param.p * err;
    iout += param.i * err;
//    pid->iout += pid->param.i * abs_zero(pid->err,0.1);
    dout = param.d * (err - last_err);

//    abs_limit(&(pid->iout), pid->param.integral_limit);
    val_limit(&iout, param.integral_higher_limit,param.integral_lower_limit);
    out = pout + iout + dout;
    abs_limit(&out, param.max_out);
    last_err = err;

    if (enable == 0) {
        out = 0;
    }
    return(out);
}

void pid::pid_reset(float kp, float ki, float kd, float max_out, float integral_limit) {
    if (kp >= 0)param.p = kp;
    if(fabsf(ki) <= 1e-6){
        param.i = 0;
        iout = 0;
    }
    else if (ki > 0)
        param.i = ki;
    if (kd >= 0)param.d = kd;
    if(max_out > 0) param.max_out = max_out;
    if(integral_limit > 0){
        param.integral_higher_limit = integral_limit;
        param.integral_lower_limit = -integral_limit;
    }else{
        param.integral_higher_limit = -integral_limit;
        param.integral_lower_limit = integral_limit;
    }

}

void pid::pid_clear() {
    iout = 0;
}