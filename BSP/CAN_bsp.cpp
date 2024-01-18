//
// Created by Tessia on 2023/12/10.
//

#include "CAN_bsp.h"

#define MAX_CAN_DEV_NUM (14 * 4) //因为过滤器一共28组，can1 can2均分，而每组过滤器有两个32位寄存器，16位list模式每个寄存器存2个id，所以max 14 * 4
can_device_receive *can_device_rx_list[2][MAX_CAN_DEV_NUM] = {0};

CAN_FilterTypeDef filter_cfg = {
        .FilterMode = CAN_FILTERMODE_IDLIST,
        .FilterScale = CAN_FILTERSCALE_16BIT,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14
};
uint32_t can1_filter[14][4] = {0};
uint32_t can1_device_num = 0;

uint32_t can2_filter[14][4] = {0};
uint32_t can2_device_num = 0;


//can1和can2消息队列
extern osMessageQueueId_t can1_send_fifoHandle;
extern osMessageQueueId_t can2_send_fifoHandle;

//can1和can2信号量
extern osSemaphoreId_t CAN1CountingSemHandle;//信号量，初始值为3，最大值为3
extern osSemaphoreId_t CAN2CountingSemHandle;//信号量，初始值为3，最大值为3


/**************************以下为can发送**********************************/

can_device_transmit::can_device_transmit(CAN_HandleTypeDef *hcan_, uint32_t id_, uint8_t len_, uint8_t *buf_data_)
               : member{hcan_,id_,len_,buf_data_} {}

can_device_transmit::can_device_transmit()
               : member{&hcan1, 0x000, 8, nullptr} {}

bool can_device_transmit::set_buf_address(uint8_t *buf_address) {
    if(buf_address != nullptr){
        member.buf_data = buf_address;
        return true;
    } else{
        return false;
    }
}

///只判断标准帧
bool can_device_transmit::set_id(uint32_t id_) {
    if(id_ > 0x7FF){
        return false;
    }else{
        member.id = id_;
        return true;
    }
}

osStatus_t can_device_transmit::can_send_msg() { //实际上是发送进队列中
    osStatus_t state;
    if (member.hcan == &hcan1) {
        state = osMessageQueuePut(can1_send_fifoHandle, this, 0, 0);
    } else {
        state = osMessageQueuePut(can2_send_fifoHandle, this, 0, 0);
    }
    return state;
}

void can1_sending_service(void *argument) { //一个task
    can_device_transmit_member msg; //消息
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for (;;) {
        osSemaphoreAcquire(CAN1CountingSemHandle,osWaitForever);//获得信号量
        osMessageQueueGet(can1_send_fifoHandle, &msg, 0, osWaitForever);//获取队列中的数据
        tx_header.StdId = msg.id;          //标准标识符
        tx_header.DLC = msg.len;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg.buf_data, &tx_mailbox);//最终调用HAL库发送
    }
}

void can2_sending_service(void *argument) { //同上
    can_device_transmit_member msg; //消息
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for (;;) {
        osSemaphoreAcquire(CAN2CountingSemHandle,osWaitForever);
        osMessageQueueGet(can2_send_fifoHandle, &msg, 0, osWaitForever);
        tx_header.StdId = msg.id;          //标准标识符
        tx_header.DLC = msg.len;
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, msg.buf_data, &tx_mailbox);
    }
}


void can_tx_complete_callback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        osSemaphoreRelease(CAN1CountingSemHandle);//释放信号量
    } else {
        osSemaphoreRelease(CAN2CountingSemHandle);
    }
}

//三个week函数
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}



/**************************以下为can接收**********************************/

can_device_receive::can_device_receive(CAN_HandleTypeDef *hcan_, uint32_t id_, can_rx_callback *callback_)
                            : hcan(hcan_), id(id_), callback(callback_), rx_data{}{
    is_callback_none = 0;

    taskENTER_CRITICAL();
    if (hcan == &hcan1 && can1_device_num < MAX_CAN_DEV_NUM) {
        filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO0;
        this->index = can1_device_num;
        can1_device_num++;
        filter_cfg.FilterBank = this->index / 4;
        can1_filter[this->index / 4][this->index % 4] = this->id;
        filter_cfg.FilterIdLow = can1_filter[this->index / 4][0] << 5;
        filter_cfg.FilterMaskIdLow = can1_filter[this->index / 4][1] << 5;
        filter_cfg.FilterIdHigh = can1_filter[this->index / 4][2] << 5;
        filter_cfg.FilterMaskIdHigh = can1_filter[this->index / 4][3] << 5;
        HAL_CAN_ConfigFilter(&hcan1, &filter_cfg);
        can_device_rx_list[0][this->index] = this;
    } else if (hcan == &hcan2 && can2_device_num < MAX_CAN_DEV_NUM) {
        filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO1;
        this->index = can2_device_num;
        can2_device_num++;
        filter_cfg.FilterBank = this->index / 4 + 14;
        can2_filter[this->index / 4][this->index % 4] = this->id;
        filter_cfg.FilterIdLow = can2_filter[this->index / 4][0] << 5;//0
        filter_cfg.FilterMaskIdLow = can2_filter[this->index / 4][1] << 5;//1
        filter_cfg.FilterIdHigh = can2_filter[this->index / 4][2] << 5;//2
        filter_cfg.FilterMaskIdHigh = can2_filter[this->index / 4][3] << 5;//3
        HAL_CAN_ConfigFilter(&hcan2, &filter_cfg);
        can_device_rx_list[1][this->index] = this;
    }
    taskEXIT_CRITICAL();

}

bool can_device_receive::can_set_callback(can_rx_callback *callback_) {
    if(callback_ != nullptr) {
        callback = callback_;
        return true;
    } else{
        return false;
    }
}


void can_device_receive::can_modify_id(uint32_t id_){
    id = id_;
    taskENTER_CRITICAL();
    if (hcan == &hcan1) {
        filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter_cfg.FilterBank = this->index / 4;
        can1_filter[this->index / 4][this->index % 4] = this->id;
        filter_cfg.FilterIdLow = can1_filter[this->index / 4][0] << 5;
        filter_cfg.FilterMaskIdLow = can1_filter[this->index / 4][1] << 5;
        filter_cfg.FilterIdHigh = can1_filter[this->index / 4][2] << 5;
        filter_cfg.FilterMaskIdHigh = can1_filter[this->index / 4][3] << 5;
        HAL_CAN_ConfigFilter(&hcan1, &filter_cfg);
        can_device_rx_list[0][this->index] = this;
    } else if (hcan == &hcan2) {
        filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO1;
        filter_cfg.FilterBank = this->index / 4 + 14;
        can2_filter[this->index / 4][this->index % 4] = this->id;
        filter_cfg.FilterIdLow = can2_filter[this->index / 4][0] << 5;//0
        filter_cfg.FilterMaskIdLow = can2_filter[this->index / 4][1] << 5;//1
        filter_cfg.FilterIdHigh = can2_filter[this->index / 4][2] << 5;//2
        filter_cfg.FilterMaskIdHigh = can2_filter[this->index / 4][3] << 5;//3
        HAL_CAN_ConfigFilter(&hcan2, &filter_cfg);
        if (3 == this->index % 4) {
            memset(can2_filter, 0, sizeof(can2_filter));
        }
        can_device_rx_list[1][this->index] = this;
    }
    taskEXIT_CRITICAL();
}


//CAN1 接收
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    uint32_t index = rx_header.FilterMatchIndex;
    can_device_receive *dev = can_device_rx_list[0][index];
    memcpy(dev->rx_data,rx_data,8);
    if (dev && dev->callback) {
        dev->callback(dev, rx_data);
    }else{
        dev->is_callback_none = 1;
    }
}

//CAN2 接收
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    uint32_t index = rx_header.FilterMatchIndex;
    can_device_receive *dev = can_device_rx_list[1][index];
    memcpy(dev->rx_data,rx_data,8);
    if (dev && dev->callback) {
        dev->callback(dev, rx_data);
    }else{
        dev->is_callback_none = 1;
    }
}