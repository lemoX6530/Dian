#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"


static const int RX_BUF_SIZE = 1024;
const uart_port_t uart_num = UART_NUM_0;  //uart0 1 2分别都能进行输出
#define TXD_PIN (GPIO_NUM_43) 
#define RXD_PIN (GPIO_NUM_44) //   ESP板子上的TX RX 分别对应的GPIO

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(uart_num, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void app_main()
{

    init();
    int count = 1;
    char* test_str = "Hello world\r\n";

    while (count<=10){


        uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));

        vTaskDelay(1000/portTICK_PERIOD_MS);
        count++;
    }



 
}




