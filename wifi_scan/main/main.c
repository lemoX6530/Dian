#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void wifi_scan_task(void *pt);
void wifi_scan_show(void *pt);
void run_on_event(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data);
void app_task(void *pt);

void run_on_event(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    ESP_LOGE("EVENT_HANDLE","BASE:%s,ID:%ld",base,id);

    switch(id)
    {
      case WIFI_EVENT_STA_START:
        ESP_LOGE("EVENT_HANDLE","WIFI_EVENT_STA_START");
        xTaskCreate(wifi_scan_task,"Wifi_scan_Task",1024*12,NULL,1,NULL);
        break;
      case WIFI_EVENT_SCAN_DONE:
        ESP_LOGE("EVENT_HANDLE","WIFI_EVENT_SCAN_DONE");
        xTaskCreate(wifi_scan_show,"Wifi_scan_Show",1024*12,NULL,1,NULL);
        break;
      default:
        break;
    }
}

void wifi_scan_task(void *pt)
{
  ESP_LOGI("WIFI", "4. Wi-Fi 扫描");
  wifi_country_t wifi_country_config = {
      .cc = "CN",
      .schan = 1,
      .nchan = 13,
  };
  ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country_config));
  ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, false));

 
  vTaskDelete(NULL);
}

void wifi_scan_show(void *pt)
{
  uint16_t ap_num = 0;
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
  ESP_LOGI("WIFI", "AP Count : %d", ap_num);

  uint16_t max_aps = 20;
  wifi_ap_record_t ap_records[max_aps];
  memset(ap_records, 0, sizeof(ap_records));

  uint16_t aps_count = max_aps;
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&aps_count, ap_records));

  ESP_LOGI("WIFI", "AP Count: %d", aps_count);

  printf("%30s %s %s %s\n", "SSID", "频道", "强度", "MAC地址");

  for (int i = 0; i < aps_count; i++)
  {
    printf("%30s  %3d  %3d  %02X-%02X-%02X-%02X-%02X-%02X\n", ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi, ap_records[i].bssid[0], ap_records[i].bssid[1], ap_records[i].bssid[2], ap_records[i].bssid[3], ap_records[i].bssid[4], ap_records[i].bssid[5]);
  }


  vTaskDelete(NULL);
}



void app_task(void *pt) 
{
  ESP_LOGI("APP_TASK","APP_TASK 创建完成");
  esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,run_on_event,NULL);
  while(1)
  {
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}


void app_main(void)
{
  ESP_LOGI("WIFI", "0. 初始化NVS存储");
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI("WIFI", "1. Wi-Fi 初始化阶段");
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));


  xTaskCreate(app_task,"App_Task",1024*12,NULL,1,NULL);

  ESP_LOGI("WIFI", "2. Wi-Fi 初始化阶段");
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  ESP_LOGI("WIFI", "3. Wi-Fi 启动阶段");
  ESP_ERROR_CHECK(esp_wifi_start());





}