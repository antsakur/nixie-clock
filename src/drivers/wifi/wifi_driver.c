#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "defines.h"
#include "wifi_driver.h"

static const char *TAG = "WiFi";

static EventGroupHandle_t wifi_event_group;
static int wifi_retry_num = 0;
static bool initial_connected;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_retry_num++;
        ESP_LOGW(TAG, "Disconnected from AP, retry %d (SSID:%s)", wifi_retry_num, ESP_WIFI_SSID);
        if (!initial_connected && wifi_retry_num == ESP_MAXIMUM_RETRY) {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP " IPSTR " (SSID:%s)", IP2STR(&event->ip_info.ip), ESP_WIFI_SSID);
        wifi_retry_num = 0;
        initial_connected = true;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_driver_connect(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    wifi_event_group = xEventGroupCreate();
    configASSERT(wifi_event_group);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = H2E_IDENTIFIER,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to SSID:%s", ESP_WIFI_SSID);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Initial connect to SSID:%s failed; retrying in background", ESP_WIFI_SSID);
    return ESP_ERR_WIFI_NOT_STARTED;
}
