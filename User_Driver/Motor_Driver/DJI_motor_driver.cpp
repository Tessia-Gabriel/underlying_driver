/**
  ******************************************************************************
  * @file           : DJI_motor_driver.cpp
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
  */



#include "DJI_motor_driver.h"

typedef struct dji_can_tx {
    CAN_HandleTypeDef *can_handle;
    uint16_t id;
    uint8_t enable;
    uint8_t buf[8];
}dji_motor_can_tx_t;

dji_motor_can_tx_t can_tx_list[] = {{&hcan1, 0x200, 0,},
                                    {&hcan1, 0x1ff, 0,},
                                    {&hcan1, 0x2ff, 0,},
                                    {&hcan2, 0x200, 0,},
                                    {&hcan2, 0x1ff, 0,},
                                    {&hcan2, 0x2ff, 0,}
};

void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data);

DJI_motor::DJI_motor(CAN_HandleTypeDef *hcan_, can_rx_callback *callback_,
                     uint32_t id_motor, DJI_MOTOR_TYPE type_, uint8_t is_reverse)
                     :can_rx_data{},motor_data{},
                      type(type_),current_to_send(0),bias_ordinal(0),
                      cmd(DJI_MOTOR_RESET_OFFSET),init_offset_f(false),is_id_false(false),

                      can_rx(hcan_,(type_==DJI_M3508||type_==DJI_M2006)?0x200+id_motor:0x204+id_motor,callback_),
                      can_tx(hcan_,(type_==DJI_M3508||type_==DJI_M2006)?(id_motor<=4?0x200:0x1ff):(id_motor<=4?0x1ff:0x2ff)),
                      velPid(0.5f, 0.1f, 10, 0.002f, 0.1f),
                      posPid(0.2f,0.1f,0.3f,0.f,0.f) {

    reverse = is_reverse; //父类继承的protected似乎无法在列表中初始化

    id_motor>0&&id_motor<9 ? id = id_motor : is_id_false = 1;
    type==DJI_GM6020?(id_motor==8?is_id_false=1:id=id_motor):id=id_motor; //判断6020id是否为8

    callback_ == nullptr ? can_rx.can_set_callback(dji_motor_data_update_callback) : can_rx.can_set_callback(callback_);


    while (init_offset_f!=1){
        osDelay(1);//在这里停止
    }

    dji_motor_can_tx_t *tx;
    if(hcan_==&hcan1){
        if(can_tx.member.id == 0x200){
            tx = &can_tx_list[0];
            can_tx_data = can_tx_list[0].buf;
        } else if(can_tx.member.id == 0x1ff){
            tx = &can_tx_list[1];
            can_tx_data = can_tx_list[1].buf;
        }else{
            tx = &can_tx_list[2];
            can_tx_data = can_tx_list[2].buf;
        }
    }else{
        if(can_tx.member.id == 0x200){
            tx = &can_tx_list[3];
            can_tx_data = can_tx_list[3].buf;
        } else if(can_tx.member.id == 0x1ff){
            tx = &can_tx_list[4];
            can_tx_data = can_tx_list[4].buf;
        }else{
            tx = &can_tx_list[5];
            can_tx_data = can_tx_list[5].buf;
        }
    }
    tx->enable = 1;
}



/*
 * 获取解算电机数据
 */
void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data) {
    DJI_motor *dji_motor = container_of(can_receive, DJI_motor, can_rx);
    memcpy(dji_motor->can_rx_data,data,8);
    dji_motor->motor_data.msg_cnt++;
    if (dji_motor->motor_data.msg_cnt > 50)
    {
        dji_motor->init_offset_f = 1;
    }
    /* initial value */
    if (dji_motor->init_offset_f == 0)
    {
        dji_motor->motor_data.ecd = (uint16_t)(dji_motor->can_rx.rx_data[0] << 8 | dji_motor->can_rx.rx_data[1]);
        dji_motor->motor_data.offset_ecd = dji_motor->motor_data.ecd;
        return;
    }

    dji_motor->motor_data.last_ecd = dji_motor->motor_data.ecd;
    dji_motor->motor_data.ecd = (uint16_t)(dji_motor->can_rx.rx_data[0] << 8 | dji_motor->can_rx.rx_data[1]);

    if (dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd > 4096)
    {
        dji_motor->motor_data.round_cnt--;
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd - 8192;
    }
    else if (dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd < -4096)
    {
        dji_motor->motor_data.round_cnt++;
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd + 8192;
    }
    else
    {
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd;
    }

    dji_motor->motor_data.total_ecd = dji_motor->motor_data.round_cnt * 8192 + dji_motor->motor_data.ecd - dji_motor->motor_data.offset_ecd;
    dji_motor->motor_data.total_round = (float)(dji_motor->motor_data.total_ecd) * ENCODER_TO_ROUND;
    dji_motor->motor_data.speed_rpm = (int16_t)(dji_motor->can_rx.rx_data[2] << 8 | dji_motor->can_rx.rx_data[3]);
    dji_motor->motor_data.given_current = (int16_t)(dji_motor->can_rx.rx_data[4] << 8 | dji_motor->can_rx.rx_data[5]);
}


void DJI_motor::dji_motor_can_tx_write() {
    uint8_t index = (id < 5 ? id : id - 4) * 2 - 1;
    taskENTER_CRITICAL();
    can_tx_data[index] = (uint8_t)(current_to_send&0xFF);
    can_tx_data[index - 1] = (uint8_t)(current_to_send>>8);
    taskEXIT_CRITICAL();
}


/**
 * 获取电机速度
 * @param void
 * @return 电机实际速度 单位rpm
 */
float DJI_motor::motor_get_speed_forward() {
    return (float)motor_data.speed_rpm;
}


/**
 * 获取电机位子
 * @param motor
 * @return 电机计入过零点检测的圈数
 */
float DJI_motor::motor_get_rounds_forward() {
    return motor_data.total_round;
}


/**
 * 设置电机电流
 * @param motor
 * @param current_normal 范围 -1～1
 */
void DJI_motor::motor_set_current_forward(float current) {
    current = current > 1.0f ? 1.0f : (current < -1.0f ? -1.0f : current);
    switch(type){
        case DJI_M3508:
            current_to_send=(int16_t)(current * MOTOR_3508_MAX_CURRENT);
            break;
        case DJI_M2006:
            current_to_send=(int16_t)(current * MOTOR_2006_MAX_CURRENT);
            break;
        case DJI_GM6020:
            current_to_send=(int16_t)(current * MOTOR_6020_MAX_CURRENT);
            break;
    }
    dji_motor_can_tx_write();
}


//speed=0.05
void DJI_motor::motor_set_speed_forward(float speed) {
    switch(type){
        case DJI_M3508:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_3508_MAX_SPEED);
            break;
        case DJI_M2006:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_2006_MAX_SPEED);
            break;
        case DJI_GM6020:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_6020_MAX_SPEED);
            break;
    }
    motor_set_current_forward(velPid.out);
}


void DJI_motor::motor_set_rounds_forward(float rounds) {
    posPid.pid_calculate(rounds, (float)(motor_get_rounds_forward()));
    motor_set_speed_forward(posPid.out);
}


bool DJI_motor::motor_reset() {
    motor_data.offset_ecd = motor_data.ecd;
    motor_data.total_ecd = 0;
    motor_data.total_round = 0;
    motor_data.round_cnt = 0;
    return(true);
}


void DJI_motor::motor_control(uint32_t cmd_) {
    cmd = (DJI_CMD)cmd_;
    switch (cmd) {
        case DJI_MOTOR_CLEAR_PID:
            velPid.pid_clear();
            posPid.pid_clear();
            break;
        case DJI_MOTOR_RESET_OFFSET:
            motor_reset();
            break;
        case DJI_MOTOR_DISABLE_OFFSET:
            motor_data.offset_ecd = 0;
    }
}


void DJI_motor_service(void *argument) {
    uint32_t tick = osKernelGetTickCount();
    can_device_transmit msg;
    for(;;){
        for(int i = 0;i<6;i++){
            if(can_tx_list[i].enable){
                msg.member.hcan = can_tx_list[i].can_handle;
                msg.member.id = can_tx_list[i].id;
                msg.member.len = 8;
                msg.member.buf_data = can_tx_list[i].buf;
                msg.can_send_msg();
            }
        }
        tick+=1;
        osDelayUntil(tick);
    }

}
