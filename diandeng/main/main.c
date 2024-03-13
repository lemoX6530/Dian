#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
uint32_t status=0;



#define LED_PIN CONFIG_LED_GPIO_NUM
#define DELAY_MS CONFIG_DELAY_TIME_MS

int count=0;

void app_main(void)
{
    gpio_reset_pin(LED_PIN);  //重启第2GPIO端口
    gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT);
    while(count<=10){
        count++;
        status = !status;  //取非
        gpio_set_level(LED_PIN,status);   //第2GPIO端口反复输出高/低电平
        ESP_LOGI("现在是","%s",status == true ? "灯亮" : "灯灭");
        vTaskDelay(DELAY_MS/10);  //delay一段时间
    }
    status=0;
    gpio_set_level(LED_PIN,status);
}
