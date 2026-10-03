#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#define WIFI_STORE_SSID_MAX 32
#define WIFI_STORE_PASS_MAX 64

bool wifi_store_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len);
esp_err_t wifi_store_save(const char *ssid, const char *pass);
esp_err_t wifi_store_erase(void);
