#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"




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

#define SAMPLING_FREQ 10 // Sampling frequency in Hz
#define DT (1.0 / SAMPLING_FREQ) // Time interval between sensor readings

static const char *TAG = "MPU6050";
const float G_accel=9.79361;

static const int RX_BUF_SIZE = 1024;
const uart_port_t uart_num = UART_NUM_0;  //uart0 1 2分别都能进行输出
#define TXD_PIN (GPIO_NUM_43) 
#define RXD_PIN (GPIO_NUM_44) //   ESP板子上的TX RX 分别对应的GPIO

#define Kp 100.0f                // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001f             // 采样周期的一半
 
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 传感器框架相对于辅助框架的四元数(初始化四元数的值
float exInt = 0, eyInt = 0, ezInt = 0;         // 由Ki缩放的积分误差项(初始化)
 
float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
 
        // 测量正常化
        norm = sqrt(ax*ax + ay*ay + az*az);

        //单位化   
        ax = ax / norm;                   
        ay = ay / norm;
        az = az / norm;      
 
        // 估计方向的重力
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
 
        // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
 
        // 积分误差比例积分增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
 
        // 调整后的陀螺仪测量
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
 
        // 整合四元数
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
 
        // 正常化四元数
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
 
        Pitch  = asin(-2*q1*q3+2*q0*q2)* 180/M_PI; 
        Roll = atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)* 180/M_PI; 
        Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)* 180/M_PI;                
        char angle_all[100]="";
        sprintf(angle_all,"Pitch:%f,Roll:%f,Yaw:%f\n",Pitch,Roll,Yaw);
        uart_write_bytes(uart_num, (const char*)angle_all, strlen(angle_all));
}

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
/**
 * @brief 初始化 I2C 总线
 */


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
    ESP_LOGI(TAG, "IIC 初始化完毕!");
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
        ESP_LOGE(TAG, "MPU6050 不在线!( %02X )", check);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 检测到在线，开始初始化...");

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
    ESP_LOGI(TAG, "MPU6050 初始化完毕!");
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


void app_main(void)
{
    i2c_init();        
    MPU6050_init();    
    init();
    ESP_LOGI(TAG, "准备采集三轴数据:");
    vTaskDelay(100);    
    while (1)
    {
        IMUupdate(get_gyro(1),get_gyro(2),get_gyro(3),get_accel(1),get_accel(2),get_accel(3));

       vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}
