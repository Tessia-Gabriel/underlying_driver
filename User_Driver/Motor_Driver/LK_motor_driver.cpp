//
// Created by Tessia on 2023/12/10.
//

#include "LK_motor_driver.h"

uint8_t read_single_motor_status[8] = {
        0x9c,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

uint8_t read_multi_motor_status[8] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

uint8_t read_pi_cmd[8] = {
        0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

uint8_t write_pi_cmd[8] = {
        0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00
}; //初始化一下，后面会改里面的值

///第一行默认为0
uint8_t LK_motor_can1_total_data[33][8] = {0};  //任务里用这里面的数据发
uint8_t LK_motor_can2_total_data[33][8] = {0};  //任务里用这里面的数据发
uint8_t LK_motor_can1_enable_list[33] = {0};         //若电机初始化成功则使能对应位置
uint8_t LK_motor_can2_enable_list[33] = {0};         //若电机初始化成功则使能对应位置

uint8_t LK_motor_can1_multi_buf[8] = {0}; //多电机can1发送，只支持四个
uint8_t LK_motor_can2_multi_buf[8] = {0}; //多电机can2发送，只支持四个
uint8_t LK_motor_can1_multi_ctrl_enable = 0;
uint8_t LK_motor_can2_multi_ctrl_enable = 0;

void LK_motor_update_data_callback(can_device_receive *can_receive, uint8_t *data);

LK_motor::LK_motor(CAN_HandleTypeDef *hcan_, can_rx_callback *callback_, uint8_t is_use_multi_ctrl_, uint32_t id_motor, LK_motor_type motor_type, uint8_t is_reverse, uint8_t is_use_motor_pid_)
         : can_rx(hcan_,0x140+id_motor,callback_), can_tx(hcan_,is_use_multi_ctrl_?0x280:0x140+id_motor), default_data_rx{},
           is_pid_send_success(false), is_init_success(false), mode(torque_ctrl), type(motor_type), is_use_multi_ctrl(is_use_multi_ctrl_),
           motor_id(id_motor), motor_data{}, power_send(0), is_use_motor_pid(is_use_motor_pid_),
           posPid(0.2f,0.1f,0.3f,0.f,0.f),
           velPid(0.5f, 0.1f, 10, 0.002f, 0.1f),
           torPid(0,0,0,0,0)                    //需重新自己赋值，因为默认使用单片机内置pid，并不需要力矩环
         {
    /** -------------------------------初始化---------------------------------- **/
    callback_ == nullptr ? can_rx.can_set_callback(LK_motor_update_data_callback) : can_rx.can_set_callback(callback_);

    default_data_tx = (hcan_ == &hcan1) ? (is_use_multi_ctrl_ ? LK_motor_can1_multi_buf : LK_motor_can1_total_data[motor_id])
                                        : (is_use_multi_ctrl_ ? LK_motor_can2_multi_buf : LK_motor_can2_total_data[motor_id]);

    can_tx.set_buf_address(default_data_tx);

    reverse = is_reverse;
    pidMode = is_use_motor_pid == 0 ? micro_pid : motor_pid;

    switch(motor_type){
        case ms_5010:
            motor_param.max_speed_dps = LK_MS_5010_MAX_SPEED_DPS;
            motor_param.max_power = LK_MS_5010_MAX_POWER;
            motor_param.max_ecd = LK_MS_5010_MAX_ECD;
            motor_param.ecd2round = LK_MS_5010_ECD2ROUND;
            break;
        case ms_6015:
            motor_param.max_speed_dps = LK_MS_6015_MAX_SPEED_DPS;
            motor_param.max_power = LK_MS_6015_MAX_POWER;
            motor_param.max_ecd = LK_MS_6015_MAX_ECD;
            motor_param.ecd2round = LK_MS_6015_ECD2ROUND;
            break;
    }

    /** -------------------------------检测电机---------------------------------- **/
    while(motor_data.msg_cnt < 50){
        motor_control(read_status);
        osDelay(1);
    }

    if(is_use_motor_pid){
        while(motor_pid_send());
    }

    is_init_success = 1;

    LK_motor_can1_enable_list[motor_id] = (hcan_ == &hcan1) ? (is_use_multi_ctrl == 1 ? 0 : 1) : 0;
    LK_motor_can2_enable_list[motor_id] = (hcan_ == &hcan2) ? (is_use_multi_ctrl == 1 ? 0 : 1) : 0;
    LK_motor_can1_multi_ctrl_enable = (hcan_ == &hcan1) ? (is_use_multi_ctrl == 1 ? 1 : 0) : 0;
    LK_motor_can2_multi_ctrl_enable = (hcan_ == &hcan2) ? (is_use_multi_ctrl == 1 ? 1 : 0) : 0;
}

bool LK_motor::motor_pid_send() {
    uint8_t wait_cnt = 0;
    uint8_t send_cnt = 0;

    switch (pidMode) {
        case motor_pid:
            is_pid_send_success = false;
            while (is_pid_send_success == false) {
                for (int cnt = 0; cnt < 5; cnt++) {
                    motor_control(write_pi);
                    osDelay(1);   //控制发送频率
                }

                while (default_data_rx[0] != 0x30) {
                    wait_cnt++;
                    motor_control(read_pi);
                    osDelay(1);
                    if (wait_cnt > 50)
                        break;
                }


                if (memcmp(default_data_rx, default_data_tx, 8) == 0) { //等于0即相等
                    is_pid_send_success = true;
                }

                send_cnt++;
                if (send_cnt > 10) {
                    break;
                }
            }
            return (is_pid_send_success);
            break;
        case micro_pid: //因为力矩控制完全不用内置，所以这里直接给true
            return true;
            break;
    }
    return true; //运行不到这里
}


void LK_motor::motor_set_rounds_forward(float rounds) {
    switch(mode){
        case pos_ctrl_2:
            default_data_tx[0] = 0xA4;
            default_data_tx[1] = 0x00;
            default_data_tx[2] = (uint16_t)(posPid.param.max_out * motor_param.max_speed_dps);
            default_data_tx[3] = (uint16_t)(posPid.param.max_out * motor_param.max_speed_dps) >> 8;
            taskENTER_CRITICAL();
            default_data_tx[4] = ((int32_t)(rounds * 36000));
            default_data_tx[5] = ((int32_t)(rounds * 36000)) >> 8;
            default_data_tx[6] = ((int32_t)(rounds * 36000)) >> 16;
            default_data_tx[7] = ((int32_t)(rounds * 36000)) >> 24;
            taskEXIT_CRITICAL();
            break;
        case speed_ctrl:        //以防忘记，保护
            mode = pos_ctrl_2;
            break;
        case torque_ctrl:
            float set = posPid.pid_calculate(rounds, motor_get_rounds_forward());
            motor_set_speed_forward(set);
            break;
    }
}

void LK_motor::motor_set_speed_forward(float speed){
    switch(mode){
        case pos_ctrl_2:        //以防忘记，保护，不用break
            mode = speed_ctrl;

        case speed_ctrl:
            default_data_tx[0] = 0xA2;
            default_data_tx[1] = 0x00;
            default_data_tx[2] = 0x00;
            default_data_tx[3] = 0x00;
            taskENTER_CRITICAL();
            default_data_tx[4] = ((int32_t)(speed * motor_param.max_speed_dps * 100));
            default_data_tx[5] = ((int32_t)(speed * motor_param.max_speed_dps * 100)) >> 8;
            default_data_tx[6] = ((int32_t)(speed * motor_param.max_speed_dps * 100)) >> 16;
            default_data_tx[7] = ((int32_t)(speed * motor_param.max_speed_dps * 100)) >> 24;
            taskEXIT_CRITICAL();
            break;
        case torque_ctrl:
            float set = velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / motor_param.max_speed_dps);
            motor_set_current_forward(set);
            break;
    }
}


void LK_motor::motor_set_current_forward(float current){
    current = current > 1.0f ? 1.0f : (current < -1.0f ? -1.0f : current);
    power_send = (int16_t)(current * motor_param.max_power);
    taskENTER_CRITICAL();
    if(is_use_multi_ctrl){
        default_data_tx[motor_id * 2] = power_send;
        default_data_tx[motor_id * 2 + 1] = power_send >> 8;
    }else{
        default_data_tx[0] = 0xA0;
        default_data_tx[1] = 0x00;
        default_data_tx[2] = 0x00;
        default_data_tx[3] = 0x00;
        default_data_tx[4] = power_send;
        default_data_tx[5] = power_send >> 8;
        default_data_tx[6] = 0x00;
        default_data_tx[7] = 0x00;
    }
    taskEXIT_CRITICAL();
}

float LK_motor::motor_get_rounds_forward() {
    return(motor_data.total_round);
}

float LK_motor::motor_get_speed_forward() {
    return(motor_data.speed_rpm);
}


///多电机仅能使用read_status
void LK_motor::motor_control(uint32_t cmd) {
    can_device_transmit can_tx_tmp(can_tx.member.hcan);
    switch(cmd){
        case read_pi:
            can_tx.set_buf_address(read_pi_cmd);
            can_tx.can_send_msg();
            can_tx.set_buf_address(default_data_tx);
            break;
        case write_pi:
            write_pi_cmd[0] = 0x32;
            write_pi_cmd[1] = 0x00;
            taskENTER_CRITICAL();
            write_pi_cmd[2] = (uint8_t)posPid.param.p;
            write_pi_cmd[3] = (uint8_t)posPid.param.i;
            write_pi_cmd[4] = (uint8_t)velPid.param.p;
            write_pi_cmd[5] = (uint8_t)velPid.param.i;
            write_pi_cmd[6] = (uint8_t)torPid.param.p;
            write_pi_cmd[7] = (uint8_t)torPid.param.i;
            can_tx.set_buf_address(write_pi_cmd);
            taskEXIT_CRITICAL();
            can_tx.can_send_msg();
            can_tx.set_buf_address(default_data_tx);
            break;
        case read_status:
            can_tx_tmp.set_id(is_use_multi_ctrl ? 0x288 : can_tx.member.id);

            if(is_use_multi_ctrl){
                can_tx_tmp.set_buf_address(read_multi_motor_status);
                read_multi_motor_status[motor_id*2] = 0x9c;
                can_tx_tmp.can_send_msg();
                read_multi_motor_status[motor_id*2] = 0;
            }else{
                can_tx_tmp.set_buf_address(read_single_motor_status);
                can_tx_tmp.can_send_msg();
            }

            break;
        case LK_disable_offset:
            motor_data.offset_ecd = 0;

    }
}


/*
 * 获取解算电机数据
 */
//位置环还有点小问题，应该是数据处理有点问题
void LK_motor_update_data_callback(can_device_receive *can_receive, uint8_t *data){
    LK_motor *lk_motor = container_of(can_receive, LK_motor, can_rx);
    memcpy(lk_motor->default_data_rx,data,8);
    lk_motor->motor_data.msg_cnt++;
    /* initial value */
    if (lk_motor->is_init_success == false)
    {
        lk_motor->motor_data.ecd = (uint16_t)(lk_motor->default_data_rx[7] << 8 | lk_motor->default_data_rx[6]);
        lk_motor->motor_data.offset_ecd = lk_motor->motor_data.ecd;
        return;
    }

    lk_motor->motor_data.last_ecd = lk_motor->motor_data.ecd;
    lk_motor->motor_data.ecd = (uint16_t)(lk_motor->default_data_rx[7] << 8 | lk_motor->default_data_rx[6]);

    if (lk_motor->motor_data.ecd - lk_motor->motor_data.last_ecd > lk_motor->motor_param.max_ecd / 2)
    {
        lk_motor->motor_data.round_cnt--;
        lk_motor->motor_data.ecd_raw_rate = lk_motor->motor_data.ecd - lk_motor->motor_data.last_ecd - lk_motor->motor_param.max_ecd;
    }
    else if (lk_motor->motor_data.ecd - lk_motor->motor_data.last_ecd < -lk_motor->motor_param.max_ecd / 2)
    {
        lk_motor->motor_data.round_cnt++;
        lk_motor->motor_data.ecd_raw_rate = lk_motor->motor_data.ecd - lk_motor->motor_data.last_ecd + lk_motor->motor_param.max_ecd;
    }
    else
    {
        lk_motor->motor_data.ecd_raw_rate = lk_motor->motor_data.ecd - lk_motor->motor_data.last_ecd;
    }

    lk_motor->motor_data.total_ecd = lk_motor->motor_data.round_cnt * lk_motor->motor_param.max_ecd + lk_motor->motor_data.ecd - lk_motor->motor_data.offset_ecd;
    lk_motor->motor_data.total_round = (float)(lk_motor->motor_data.total_ecd) * lk_motor->motor_param.ecd2round;
    lk_motor->motor_data.speed_rpm = (int16_t)(lk_motor->default_data_rx[5] << 8 | lk_motor->default_data_rx[4]);
    lk_motor->motor_data.power = (int16_t)(lk_motor->default_data_rx[3] << 8 | lk_motor->default_data_rx[2]);
    lk_motor->motor_data.temperature = (int8_t)(lk_motor->default_data_rx[1]);
}

bool LK_motor::motor_reset() {
    motor_data.offset_ecd = motor_data.ecd;
    motor_data.total_ecd = 0;
    motor_data.total_round = 0;
    motor_data.round_cnt = 0;
    return(true);
}

void LK_motor_service(void *argument){
    can_device_transmit LK_motor_can1_service(&hcan1);
    can_device_transmit LK_motor_can2_service(&hcan2);
    for(;;){
        for(int id=1;id<=32;id++){

            if(LK_motor_can1_enable_list[id] == 1) {
                LK_motor_can1_service.set_id(0x140 + id);
                LK_motor_can1_service.set_buf_address(LK_motor_can1_total_data[id]);
                LK_motor_can1_service.can_send_msg();
            }

            if(LK_motor_can2_enable_list[id] == 1) {
                LK_motor_can2_service.set_id(0x140 + id);
                LK_motor_can2_service.set_buf_address(LK_motor_can2_total_data[id]);
                LK_motor_can2_service.can_send_msg();
            }


        }
        osDelay(1);
    }
}