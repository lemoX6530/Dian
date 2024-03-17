
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

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



#define ESP_MAXIMUM_RETRY 10
#define ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASSWORD CONFIG_ESP_WIFI_PASSWORD
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define TCP_SERVER_ADRESS       CONFIG_TCP_SERVER_ADRESS     //作为client，要连接TCP服务器地址
#define TCP_SERVER_PORT          CONFIG_TCP_SERVER_PORT               //服务端端口
#define TCP_CLIENT_PORT          CONFIG_TCP_CLIENT_PORT
static TaskHandle_t xHandleTaskTcpClinent = NULL;
static TaskHandle_t xHandleTaskTcpServer = NULL;

//任务堆栈大小，主要是函数嵌套参数入栈和局部变量的内存
#define TcpClinent_TASK_STACK_SIZE      8192
#define TcpServer_TASK_STACK_SIZE       8192

//任务优先级，越大越高，跟ucos相反
#define TcpClinent_TASK_PRIO            2
#define TcpServer_TASK_PRIO             3

extern EventGroupHandle_t xCreatedEventGroup_WifiConnect;



static const char *TAG = "WIFI_TCP";

//事件标志组
EventGroupHandle_t xCreatedEventGroup_WifiConnect = NULL;

static int retry_num = 0;
static int client_connect_socket = 0;                     //客户端连接socket
static int server_socket = 0;                           //服务器创建的socket
static int server_connect_socket = 0;                   //有客户端连接到服务器的socket

//函数声明
static esp_err_t CreateTcpClient(const char *ip,uint16_t port);
static esp_err_t CreateTcpServer(bool isCreateServer,uint16_t port);
static void vTaskTcpClient(void *pvParameters);
static void vTaskTcpServer(void *pvParameters);
void AppTaskCreate(void);

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
            send(client_connect_socket, databuff, len, 0);
            ESP_LOGI("vTaskTcpClient","receiving and sending message");
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
            send(server_connect_socket, databuff, len, 0);
            ESP_LOGI("vTaskTcpServer","receiving and sending message");
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
    ESP_LOGI(TAG, "event_base: %s  ; event_id : %ld",event_base,event_id);
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
            ESP_LOGI(TAG, "retry to connect to the AP");
        } 
        else 
        {
            xEventGroupSetBits(xCreatedEventGroup_WifiConnect, WIFI_FAIL_BIT);
            xEventGroupClearBits(xCreatedEventGroup_WifiConnect, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)//获得IP
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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

    ESP_LOGI(TAG, "wifi_init_sta finished.");
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
    //初始化NVS
   esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    //创建事件标志组，用于等待wifi连接
    xCreatedEventGroup_WifiConnect = xEventGroupCreate();
    //WIFI sta初始化
    wifi_init_sta();
    AppTaskCreate();
}
