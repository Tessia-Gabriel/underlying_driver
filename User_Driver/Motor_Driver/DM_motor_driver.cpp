/**
  ******************************************************************************
  * @file           : DM_motor_driver.cpp
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2024/1/16
  ******************************************************************************
  */



#include "DM_motor_driver.h"

//can需要5V供电 同时控制数据一发一反馈，不发不反馈

//目前假设最大32个电机，0行不用
uint8_t DM_motor_cnt = 0;
uint8_t DM_motor_can1_total_data[33][8] = {0};  //任务里用这里面的数据发
uint8_t DM_motor_can2_total_data[33][8] = {0};  //任务里用这里面的数据发
uint8_t DM_motor_can1_tx_id[33] = {0};  //存id
uint8_t DM_motor_can2_tx_id[33] = {0};  //存id
uint8_t DM_motor_can1_enable_list[33] = {0};         //若电机初始化成功则使能对应位置
uint8_t DM_motor_can2_enable_list[33] = {0};         //若电机初始化成功则使能对应位置

uint8_t array_enable_motor[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
uint8_t array_disable_motor[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd};
uint8_t array_save_zero_offset[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe};

void dm_motor_rx_data_update_callback(can_device_receive *can_receive, uint8_t *data);

DM_motor::DM_motor(CAN_HandleTypeDef *hcan_, can_rx_callback *callback_,
        uint32_t rx_id, uint32_t tx_id, motor_type type_, dm_motor_mode mode_, uint8_t is_reverse)
        :is_lost(false),init_offset(false),type(type_),mode(mode_),hcan(hcan_),
         raw{},data{},ctrl{},tx_buff{},

         can_rx(hcan_,rx_id,callback_),
         can_tx(hcan_,tx_id+0x100*mode_),
         velPid(1.0f, 0.2, 0.6f, 0.0001, 0.1),
         posPid(1.0f, 0.1, 8.0f, 0, 1.5){

    /** -------------------------------初始化参数---------------------------------- **/
    reverse = is_reverse;

    callback_ == nullptr ? can_rx.can_set_callback(dm_motor_rx_data_update_callback) : can_rx.can_set_callback(callback_);
    DM_motor_cnt++;
    motor_num = DM_motor_cnt;
    default_data_tx = (hcan_ == &hcan1) ? DM_motor_can1_total_data[motor_num] : DM_motor_can2_total_data[motor_num];
    can_tx.set_buf_address(default_data_tx);

    (hcan_ == &hcan1) ? DM_motor_can1_tx_id[motor_num] = tx_id+0x100*mode_ : DM_motor_can2_tx_id[motor_num] = tx_id+0x100*mode_;



    /** -------------------------------检测电机---------------------------------- **/

    memcpy(can_tx.member.buf_data, array_enable_motor, 8);
    uint8_t cnt = 0;
    while(cnt<10){
        can_tx.can_send_msg();
        cnt++;
    }

    while(init_offset == 0){
        motor_set_current_forward(0);
        can_tx.can_send_msg();
    }

    DM_motor_can1_enable_list[motor_num] = (hcan_ == &hcan1) ? 1 : 0;
    DM_motor_can2_enable_list[motor_num] = (hcan_ == &hcan2) ? 1 : 0;

}


float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits){
/// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

//不断调用
__RAM_FUNC
void DM_motor::set_dm_ctrl_to_can_tx_buff(){
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint8_t *pbuf,*vbuf;
    switch (mode) {
        case DM_MIT:
            pos_tmp = float_to_uint(ctrl.p_des, -DM_J4310_2EC_P_MAX, DM_J4310_2EC_P_MAX, 16);
            vel_tmp = float_to_uint(ctrl.v_des, -DM_J4310_2EC_V_MAX, DM_J4310_2EC_V_MAX, 12);
            tor_tmp = float_to_uint(ctrl.torq, -DM_J4310_2EC_T_MAX, DM_J4310_2EC_T_MAX, 12);
            kp_tmp = float_to_uint(ctrl.kp, DM_J4310_2EC_KP_MIN, DM_J4310_2EC_KP_MAX, 12);
            kd_tmp = float_to_uint(ctrl.kd, DM_J4310_2EC_KD_MIN, DM_J4310_2EC_KD_MAX, 12);

            can_tx.member.buf_data[0] = (pos_tmp >> 8);
            can_tx.member.buf_data[1] = pos_tmp;
            can_tx.member.buf_data[2] = (vel_tmp >> 4);
            can_tx.member.buf_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
            can_tx.member.buf_data[4] = kp_tmp;
            can_tx.member.buf_data[5] = (kd_tmp >> 4);
            can_tx.member.buf_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
            can_tx.member.buf_data[7] = tor_tmp;
            break;
        case DM_PV:
            pbuf=(uint8_t *)&ctrl.p_des;
            vbuf=(uint8_t *)&ctrl.v_des;
            can_tx.member.buf_data[0] = *pbuf;
            can_tx.member.buf_data[1] = *(pbuf+1);
            can_tx.member.buf_data[2] = *(pbuf+2);
            can_tx.member.buf_data[3] = *(pbuf+3);
            can_tx.member.buf_data[4] = *vbuf;
            can_tx.member.buf_data[5] = *(vbuf+1);
            can_tx.member.buf_data[6] = *(vbuf+2);
            can_tx.member.buf_data[7] = *(vbuf+3);
            break;
        case DM_VO:
            vbuf=(uint8_t *)&ctrl.v_des;
            can_tx.member.buf_data[0] = *vbuf;
            can_tx.member.buf_data[1] = *(vbuf+1);
            can_tx.member.buf_data[2] = *(vbuf+2);
            can_tx.member.buf_data[3] = *(vbuf+3);
            break;
        default:
            break;
    }
}



float DM_motor::motor_get_rounds_forward() {
    return data.total_rounds;
}

float DM_motor::motor_get_speed_forward() {
    return data.current_speed;
}

void DM_motor::motor_set_current_forward(float current) {
    ctrl.kp = 0;
    ctrl.kd = 0;
    ctrl.v_des = 0;
    ctrl.p_des = 0;
    current = current > 1.0f ? 1.0f : (current < -1.0f ? -1.0f : current);
    ctrl.torq = current * DM_J4310_2EC_T_MAX;
    set_dm_ctrl_to_can_tx_buff();
}


void DM_motor::motor_set_speed_forward(float speed) {
    velPid.pid_calculate(speed, motor_get_speed_forward());
    motor_set_current_forward(velPid.out);
}

void DM_motor::motor_set_rounds_forward(float rounds) {
    posPid.pid_calculate(rounds, motor_get_rounds_forward());
    motor_set_speed_forward(posPid.out);
}




void dm_motor_rx_data_update_callback(can_device_receive *can_receive, uint8_t *data){
    DM_motor *dm_motor = container_of(can_receive, DM_motor, can_rx);
    dm_motor_data *ptr = &(dm_motor->data);
    dm_motor_raw_data *raw = &(dm_motor->raw);

    raw->id = data[0]&0xF;
    raw->err = data[0]>>4;
    //若id>0x0f,则解算err出错
    raw->pos = (data[1]<<8)|data[2];
    raw->vel =(data[3]<<4)|(data[4]>>4);
    raw->torque = ((data[4]&0xF)<<8)|data[5];
    raw->t_mos = data[6];
    raw->t_rotor = data[7];

    //get_offset
    ptr->msg_cnt++;
    if (ptr->msg_cnt > 50) {
        dm_motor->init_offset = true;
    }
    /* initial value */
    if (dm_motor->init_offset == false)
    {
        if(dm_motor->type==DM_J4310_2EC){
            ptr->current_speed = //-1-1
                    uint_to_float(raw->vel, -DM_J4310_2EC_V_MAX, DM_J4310_2EC_V_MAX, 12) / DM_J4310_2EC_V_MAX; // (-30.0,30.0)
        }

        ptr->current_round =// 0 - 2*P_MAX
                (uint_to_float(raw->pos, -DM_J4310_2EC_P_MAX, DM_J4310_2EC_P_MAX, 16)+DM_J4310_2EC_P_MAX) * RAD2ROUND; // (-3.14,3.14)

        ptr->offset_round = ptr->current_round;
        return;
    }

    //误差改小,溢出cnt
    ptr->last_round = ptr->current_round;//电机编码反馈值
    ptr->current_round =  // (0 - 2*P_MAX)/(2*PI)
            (uint_to_float(raw->pos, -DM_J4310_2EC_P_MAX, DM_J4310_2EC_P_MAX, 16) + DM_J4310_2EC_P_MAX) * RAD2ROUND; // (-3.14,3.14)
    if (ptr->current_round - ptr->last_round > DM_J4310_2EC_P_MAX*RAD2ROUND){
        ptr->round_cnt = ptr->round_cnt - 1;
        ptr->delta_rounds = ptr->current_round - ptr->last_round - (2.0f*DM_J4310_2EC_P_MAX*RAD2ROUND);
    }
    else if (ptr->current_round - ptr->last_round < -DM_J4310_2EC_P_MAX*RAD2ROUND){
        ptr->round_cnt = ptr->round_cnt + 1;
        ptr->delta_rounds = ptr->current_round - ptr->last_round + (2.0f*DM_J4310_2EC_P_MAX*RAD2ROUND);
    }
    else{
        ptr->delta_rounds = ptr->current_round - ptr->last_round;
    }
    ptr->total_rounds = (float) ptr->round_cnt * (2.0f*DM_J4310_2EC_P_MAX*RAD2ROUND) + ptr->current_round - ptr->offset_round;

    ptr->last_speed = ptr->current_speed;
    if(dm_motor->type==DM_J4310_2EC) {
        ptr->current_speed = //-1-1
                uint_to_float(raw->vel, -DM_J4310_2EC_V_MAX, DM_J4310_2EC_V_MAX, 12) / DM_J4310_2EC_V_MAX; // (-30.0,30.0)
    }

    ptr->torque = uint_to_float(raw->torque, -DM_J4310_2EC_T_MAX, DM_J4310_2EC_T_MAX, 12); // (-10.0,10.0)

//    /*#### add enable can it again to solve can receive only one ID problem!!! ####**/
//    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}


bool DM_motor::motor_reset() {
    data.offset_ecd = data.ecd;
    data.total_ecd = 0;
    data.total_rounds = 0;
    data.round_cnt = 0;
    return(true);
}


void DM_motor::motor_control(uint32_t cmd) {
    switch(cmd){
        case enable_motor:
            memcpy(can_tx.member.buf_data, array_enable_motor, 8);
            break;

        case disable_motor:
            memcpy(can_tx.member.buf_data, array_disable_motor, 8);
            break;

        case save_zero_offset:
            memcpy(can_tx.member.buf_data, array_save_zero_offset, 8);
            break;

        case clear_pid:
            velPid.pid_clear();
            posPid.pid_clear();
            break;

        case disable_offset:
            data.offset_ecd = 0;
            data.offset_round = 0;
            break;
    }
}



void DM_motor_service(void *argument){
    can_device_transmit LK_motor_can1_service(&hcan1);
    can_device_transmit LK_motor_can2_service(&hcan2);
    for(;;){
        for(int i=1;i<=32;i++){

            if(DM_motor_can1_enable_list[i] == 1) {
                LK_motor_can1_service.set_id(DM_motor_can1_tx_id[i]);
                LK_motor_can1_service.set_buf_address(DM_motor_can1_total_data[i]);
                LK_motor_can1_service.can_send_msg();
            }

            if(DM_motor_can2_enable_list[i] == 1) {
                LK_motor_can2_service.set_id(DM_motor_can1_tx_id[i]);
                LK_motor_can2_service.set_buf_address(DM_motor_can2_total_data[i]);
                LK_motor_can2_service.can_send_msg();
            }


        }
        osDelay(1);
    }
}




