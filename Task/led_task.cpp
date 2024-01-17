//
// Created by Tessia on 2023/12/9.
//

#include "led_task.h"
#include "cmsis_os.h"
#include "main.h"

void LED_task(void *argument){
    for(;;){
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        osDelay(200);
    }
}