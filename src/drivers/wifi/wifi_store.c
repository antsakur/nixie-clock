#include <string.h>

#include "nvs.h"
#include "esp_log.h"

#include "wifi_store.h"

static const char *TAG = "WiFiStore";
static const char *NS = "wifi_cfg";

bool wifi_store_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    if (!ssid || ssid_len < 2 || !pass || pass_len < 2) {
        return false;
    }
    ssid[0] = '\0';
    pass[0] = '\0';

    nvs_handle_t handle;
    if (nvs_open(NS, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t ssid_size = ssid_len;
    size_t pass_size = pass_len;
    esp_err_t ssid_err = nvs_get_str(handle, "ssid", ssid, &ssid_size);
    esp_err_t pass_err = nvs_get_str(handle, "pass", pass, &pass_size);
    nvs_close(handle);

    if (ssid_err != ESP_OK || ssid[0] == '\0') {
        ESP_LOGI(TAG, "No saved Wi-Fi credentials");
        return false;
    }
    if (pass_err != ESP_OK) {
        pass[0] = '\0';
    }
    ESP_LOGI(TAG, "Loaded saved SSID:%s", ssid);
    return true;
}

esp_err_t wifi_store_save(const char *ssid, const char *pass)
{
    if (!ssid || ssid[0] == '\0' || strlen(ssid) > WIFI_STORE_SSID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pass && strlen(pass) > WIFI_STORE_PASS_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, "pass", pass ? pass : "");
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved SSID:%s", ssid);
    }
    return err;
}

esp_err_t wifi_store_erase(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    nvs_erase_all(handle);
    err = nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Erased saved Wi-Fi credentials");
    return err;
}
