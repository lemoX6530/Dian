
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <math.h>
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"



#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_SCL_IO 5
#define I2C_MASTER_SDA_IO 4
#define ESP_MAXIMUM_RETRY 10
#define ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASSWORD CONFIG_ESP_WIFI_PASSWORD
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define TCP_SERVER_ADRESS       CONFIG_TCP_SERVER_ADRESS     //作为client，要连接TCP服务器地址
#define TCP_SERVER_PORT          CONFIG_TCP_SERVER_PORT               //服务端端口
#define TCP_CLIENT_PORT          CONFIG_TCP_CLIENT_PORT

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_SCL_IO 5
#define I2C_MASTER_SDA_IO 4

#define MPU_ADDR 0x68             // MPU6050 设备地址
#define MPU_CMD_WHO_AM_I 0x75     // MPU6050 设备确认寄存器
#define MPU_CMD_PWR_MGMT_1 0x6B   // 电源管理寄存器
#define MPU_CMD_SMPLRT_DIV 0x19   // 陀螺仪采样率分频器寄存器
#define MPU_CMD_CONFIG 0x1A       // 数字低通滤波器配置寄存器
#define MPU_CMD_GYRO_CONFIG 0x1B  // 陀螺仪配置寄存器
#define MPU_CMD_ACCEL_CONFIG 0x1C // 加速度传感器配置寄存器

#define MPU_CMD_ACCEL_XOUT_H 0x3B // 加速计X轴高字节数据寄存器
#define MPU_CMD_ACCEL_YOUT_H 0x3D
#define MPU_CMD_ACCEL_ZOUT_H 0x3F

#define MPU_CMD_GYRO_XOUT_H 0x43 // 加速计X轴高字节数据寄存器
#define MPU_CMD_GYRO_YOUT_H 0x45
#define MPU_CMD_GYRO_ZOUT_H 0x47





static TaskHandle_t xHandleTaskTcpClinent = NULL;
static TaskHandle_t xHandleTaskTcpServer = NULL;

//任务堆栈大小，主要是函数嵌套参数入栈和局部变量的内存
#define TcpClinent_TASK_STACK_SIZE      8192
#define TcpServer_TASK_STACK_SIZE       8192

//任务优先级，越大越高，跟ucos相反
#define TcpClinent_TASK_PRIO            2
#define TcpServer_TASK_PRIO             3


extern EventGroupHandle_t xCreatedEventGroup_WifiConnect;

static const char *WIFI_TAG = "WIFI_TCP";
static const char *MPU_TAG = "MPU6050";

//事件标志组
EventGroupHandle_t xCreatedEventGroup_WifiConnect = NULL;

static int retry_num = 0;
static int client_connect_socket = 0;                     //客户端连接socket
static int server_socket = 0;                           //服务器创建的socket
static int server_connect_socket = 0;                   //有客户端连接到服务器的socket

const float G_accel=9.79361;

char accel_all[100]="";
char gyro_all[100]="";



//函数声明
static esp_err_t CreateTcpClient(const char *ip,uint16_t port);
static esp_err_t CreateTcpServer(bool isCreateServer,uint16_t port);
static void vTaskTcpClient(void *pvParameters);
static void vTaskTcpServer(void *pvParameters);
void AppTaskCreate(void);

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


void i2c_init()
{
    // 配置 IIC 总线参数
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,               // 主机方式启动IIC
        .sda_io_num = I2C_MASTER_SDA_IO,       // 设置数据引脚编号
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // 使能 SDA 上拉电阻
        .scl_io_num = I2C_MASTER_SCL_IO,       // 设置始终引脚编号
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // 使能 SCL 上拉电阻
        .master.clk_speed = I2C_MASTER_FREQ_HZ // 设置主频
    };
    // 初始化 IIC 总线
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    // 安装 IIC 设备，因为 slv_rx_buf_len 和 slv_tx_buf_len 中主机模式下会忽略，所以不配置，中断标志位也不做分配
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(MPU_TAG, "IIC 初始化完毕!");
}

void MPU6050_init()
{
    uint8_t check;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送地址，以及写指令，命令之后需要带ACK
    i2c_master_write_byte(cmd, MPU_CMD_WHO_AM_I, true);                   // 发送 WHO_MI_I 寄存器地址 0x75
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &check, I2C_MASTER_LAST_NACK);              // 读取数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);                                             // 删除指令
    if (check != 0x68)
    {
        ESP_LOGE(MPU_TAG, "MPU6050 不在线!( %02X )", check);
        return;
    }
    ESP_LOGI(MPU_TAG, "MPU6050 检测到在线，开始初始化...");

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_PWR_MGMT_1, true);                 // 写入电源管理和复位控制
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（设置陀螺仪采样率分频器）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_SMPLRT_DIV, true);                 // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x07, true);                               // 写入寄存器数据 Sample rate = 1kHz/(7+1) = 125Hz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（数字低通滤波器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_CONFIG, true);                     // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope：260Hz 0ms，Accelerometer：256Hz 0.98ms 8Khz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（陀螺仪配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_GYRO_CONFIG, true);                // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope: +/- 250 dps
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（加速度传感器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_CONFIG, true);               // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Accelerometer: +/- 2g
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(MPU_TAG, "MPU6050 初始化完毕!");
}
/**
 * @brief 获得 X 轴加速度
 * @return 返回 X 轴加速度
 */
float get_accel(int direction)
{
    int MPU_CMD_ACCEL_OUT_H=0;
    switch (direction){
        case 1:
            MPU_CMD_ACCEL_OUT_H=MPU_CMD_ACCEL_XOUT_H;
            break;
        case 2:
            MPU_CMD_ACCEL_OUT_H=MPU_CMD_ACCEL_YOUT_H;
            break;
        case 3:
            MPU_CMD_ACCEL_OUT_H=MPU_CMD_ACCEL_ZOUT_H;
            break;
    }
    union
    {
        uint8_t bytes[4];
        int16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_OUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    float data_a=data.value+0.0;
    data_a=data_a/65536*4*G_accel;
    return data_a;
}

float get_gyro(int direction)
{
    int MPU_CMD_GYRO_OUT_H=0;
    switch (direction){
        case 1:
            MPU_CMD_GYRO_OUT_H=MPU_CMD_GYRO_XOUT_H;
            break;
        case 2:
            MPU_CMD_GYRO_OUT_H=MPU_CMD_GYRO_YOUT_H;
            break;
        case 3:
            MPU_CMD_GYRO_OUT_H=MPU_CMD_GYRO_ZOUT_H;
            break;
    }
    union
    {
        uint8_t bytes[4];
        int16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_GYRO_OUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    float data_g=data.value+0.0;
    data_g=data_g/65536*4*500;
    return data_g;
}

/***********************************************************************
* 函数:  
* 描述:   创建tcp接收发送任务，给外部使用
* 参数:
* 返回: 无
* 备注：
************************************************************************/
void AppTaskCreate(void)
{
    xTaskCreate(vTaskTcpClient,                    //任务函数 
            "vTaskTcpClient",                      // 任务名 
            TcpClinent_TASK_STACK_SIZE,             // 任务栈大小，单位 word，也就是 4 字节 
            NULL,                                   //任务参数 
            TcpClinent_TASK_PRIO,                   //任务优先级
            &xHandleTaskTcpClinent);    
    xTaskCreate(vTaskTcpServer,                     //任务函数 
            "vTaskTcpServer",                       // 任务名 
            TcpServer_TASK_STACK_SIZE,              // 任务栈大小，单位 word，也就是 4 字节 
            NULL,                                   //任务参数 
            TcpServer_TASK_PRIO,                    //任务优先级
            &xHandleTaskTcpServer); 
}
/***********************************************************************
* 函数:  
* 描述:   创建tcp客户端连接
* 参数:ip：IP地址，port：端口号
* 返回: 无
* 备注：
************************************************************************/
static esp_err_t CreateTcpClient(const char *ip,uint16_t port)
{
    static struct sockaddr_in server_addr;                  //server地址
    //等待连接成功，或已经连接有断开连接，此函数会一直阻塞，只有有连接
   EventBits_t bits =  xEventGroupWaitBits(xCreatedEventGroup_WifiConnect,// 事件标志组句柄 
                WIFI_CONNECTED_BIT, // 等待bit0和bit1被设置 
                pdFALSE,                            //TRUE退出时bit0和bit1被清除，pdFALSE退出时bit0和bit1不清除
                pdFALSE,                            //设置为pdTRUE表示等待bit1和bit0都被设置,pdFALSE表示等待bit1或bit0其中一个被设置
                portMAX_DELAY);                     //等待延迟时间，一直等待 

    if (bits & WIFI_CONNECTED_BIT) 
    {
        //新建socket
        client_connect_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (client_connect_socket < 0)
        {
            close(client_connect_socket);
            return ESP_FAIL;
        }
        //配置连接服务器信息
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr(ip);
        //连接服务器
        if (connect(client_connect_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            close(client_connect_socket);
            return ESP_FAIL;
        }
        return ESP_OK;
    } 
    else
    {
        return ESP_FAIL;
    }
}
/***********************************************************************
* 函数:  
* 描述:   客户端任务
* 参数:
* 返回: 无
* 备注：
************************************************************************/
static void vTaskTcpClient(void *pvParameters)
{
    int len = 0;            //长度

    int fail_count = 0;
    uint8_t databuff[512];    //缓存
    //创建tcp客户端
    if (CreateTcpClient(TCP_SERVER_ADRESS,TCP_SERVER_PORT) == ESP_OK)
    {
        ESP_LOGI("vTaskTcpClient", "connect OK");
    }
    else
    {
        ESP_LOGI("vTaskTcpClient", "connect FAIL,Retrying");
        while (CreateTcpClient(TCP_SERVER_ADRESS,TCP_SERVER_PORT) != ESP_OK && fail_count<=ESP_MAXIMUM_RETRY){
            ESP_LOGI("vTaskTcpClient", "connect FAIL,Retrying");
            fail_count++;
        }
        ESP_LOGI("vTaskTcpClient", "connect FAIL,Aborting");
    }
    for (;;)
    {
        //读取接收数据
        len = recv(client_connect_socket, databuff, sizeof(databuff), 0);//阻塞函数
        if (len > 0)//接收到数据
        {
            while (1){
                sprintf(accel_all,"X_ACCEL:%f,Y_ACCEL:%f,Z_ACCEL:%f\n",get_accel(1),get_accel(2),get_accel(3));
                sprintf(gyro_all,"X_GYRO:%f,Y_GYRO:%f,Z_GYRO:%f\n",get_gyro(1),get_gyro(2),get_gyro(3));
                
                send(client_connect_socket, accel_all, sizeof(accel_all), 0);
                send(client_connect_socket, gyro_all, sizeof(gyro_all), 0);
                vTaskDelay(50 / portTICK_PERIOD_MS);

            }
        }
        else
        {
            //断开连接
            if (CreateTcpClient(TCP_SERVER_ADRESS,TCP_SERVER_PORT)== ESP_OK)
            {
                ESP_LOGI("vTaskTcpClient", "connect OK");
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/***********************************************************************
* 函数:  
* 描述:   创建服务端监听任务
* 参数:isCreatServer：true创建socket并监听，false只监听，port监听的端口号
* 返回: 无
* 备注：
************************************************************************/
static esp_err_t CreateTcpServer(bool isCreateServer,uint16_t port)
{
    static struct sockaddr_in client_addr;   
    static unsigned long int socklen = sizeof(client_addr);      //地址长度
    //等待连接成功，或已经连接有断开连接，此函数会一直阻塞，只有有连接
   EventBits_t bits =  xEventGroupWaitBits(xCreatedEventGroup_WifiConnect,// 事件标志组句柄 
                WIFI_CONNECTED_BIT,              // 等待bit0和bit1被设置 
                pdFALSE,                            //TRUE退出时bit0和bit1被清除，pdFALSE退出时bit0和bit1不清除
                pdFALSE,                            //设置为pdTRUE表示等待bit1和bit0都被设置,pdFALSE表示等待bit1或bit0其中一个被设置
                portMAX_DELAY);                     //等待延迟时间，一直等待 

    if (bits & WIFI_CONNECTED_BIT) 
    {
        if (isCreateServer == true)
        {
            server_socket = socket(AF_INET, SOCK_STREAM, 0);
            if (server_socket < 0)
            {
                close(server_socket);
                return ESP_FAIL;
            }
            //配置新建server socket参数
            client_addr.sin_family = AF_INET;
            client_addr.sin_port = htons(port);
            client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
            //bind:地址的绑定
            if (bind(server_socket, (struct sockaddr *)&client_addr, sizeof(client_addr)) < 0)
            {
                //bind失败后，关闭新建的socket，等待下次新建
                close(server_socket);
                return ESP_FAIL;
            }
        }
        
        //listen，下次时，直接监听
        if (listen(server_socket, 1) < 0)
        {
            //listen失败后，关闭新建的socket，等待下次新建
            close(server_socket);
            return ESP_FAIL;
        }
        //accept，搜寻全连接队列
        server_connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
        if (server_connect_socket < 0)
        {
            //accept失败后，关闭新建的socket，等待下次新建
            close(server_socket);
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }

}
/***********************************************************************
* 函数:  
* 描述:   服务端任务
* 参数:
* 返回: 无
* 备注：
************************************************************************/
static void vTaskTcpServer(void *pvParameters)
{
    int len = 0;            //长度
    
    uint8_t databuff[512];    //缓存
    //创建tcp客户端
    CreateTcpServer(true,TCP_CLIENT_PORT);
    for (;;)
    {
        //读取接收数据
        len = recv(server_connect_socket, databuff, sizeof(databuff), 0);//阻塞函数
        if (len > 0)//接收到数据
        {
            while (1){
                sprintf(accel_all,"X_ACCEL:%fm/s^2,Y_ACCEL:%fm/s^2,Z_ACCEL:%fm/s^2 \n",get_accel(1),get_accel(2),get_accel(3));
                sprintf(gyro_all,"X_GYRO:%f°/s,Y_GYRO:%f°/s,Z_GYRO:%f°/s\n",get_gyro(1),get_gyro(2),get_gyro(3));
                send(client_connect_socket, accel_all, sizeof(accel_all), 0);
                send(client_connect_socket, gyro_all, sizeof(gyro_all), 0);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        
        else
        {
            //断开连接
            CreateTcpServer(false,TCP_CLIENT_PORT);  
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


/***********************************************************************
* 函数:  
* 描述:   wifi 回调函数
* 参数:
* 返回: 无
* 备注：
************************************************************************/
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    ESP_LOGI(WIFI_TAG, "event_base: %s  ; event_id : %ld",event_base,event_id);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();//开始连接
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) //wifi断开时
    {
        if (retry_num < ESP_MAXIMUM_RETRY) //重新连接次数
        {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } 
        else 
        {
            xEventGroupSetBits(xCreatedEventGroup_WifiConnect, WIFI_FAIL_BIT);
            xEventGroupClearBits(xCreatedEventGroup_WifiConnect, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(WIFI_TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)//获得IP
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        char IPS[100]="";
        sprintf(IPS,"%d.%d.%d.%d\n",IP2STR(&event->ip_info.ip));
        uart_write_bytes(uart_num, (const char*)IPS, strlen(IPS));
        retry_num = 0;
        xEventGroupSetBits(xCreatedEventGroup_WifiConnect, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(xCreatedEventGroup_WifiConnect, WIFI_FAIL_BIT);
    }
}
/***********************************************************************
* 函数:  
* 描述:   wifi sta初始化代码相对比较固定，一般不需要做修改
* 参数:
* 返回: 无
* 备注：
************************************************************************/
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());//初始化TCP / IP堆栈和esp-netif
    ESP_ERROR_CHECK(esp_event_loop_create_default()); //创建默认event loop
    esp_netif_create_default_wifi_sta();//创建默认的WIFI STA。

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();//给wifi_init_config_t结构体初始化系统默认的数据
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));//使用默认的参数初始化wifi

    //注册wifi 连接过程中回调函数
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    //设置账户和密码,在menuconfig中设置
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASSWORD
            //.ssid = "ssid",
            //.password = "password"
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );//设置为sta模式
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );//启动

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");
}
/***********************************************************************
* 函数:  
* 描述:   主函数
* 参数:
* 返回: 无
* 备注：
************************************************************************/
void app_main()
{    
    init();
    //初始化NVS
   esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    i2c_init();        
    MPU6050_init();

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    //创建事件标志组，用于等待wifi连接
    xCreatedEventGroup_WifiConnect = xEventGroupCreate();
    //WIFI sta初始化
    wifi_init_sta();
    AppTaskCreate();
}
