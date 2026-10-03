#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

typedef void (*wifi_got_ip_cb_t)(void);

void wifi_driver_init(void);
void wifi_driver_set_got_ip_cb(wifi_got_ip_cb_t cb);
esp_err_t wifi_driver_start_sta(const char *ssid, const char *password);
// Scan and join while the setup AP stays up. Does not write NVS.
// On ESP_OK the STA has an IP; call wifi_driver_provision_commit() after saving.
// On error, err_msg (if non-NULL) explains why and the setup AP is still running.
esp_err_t wifi_driver_check_credentials(const char *ssid, const char *password,
                                        char *err_msg, size_t err_msg_len);
esp_err_t wifi_driver_provision(const char *ssid, const char *password, bool allow_hidden,
                                char *err_msg, size_t err_msg_len);
void wifi_driver_provision_commit(void);
esp_err_t wifi_driver_start_ap(void);
esp_err_t wifi_driver_enter_setup_mode(void);
bool wifi_driver_sta_connected(void);
void wifi_driver_get_sta_ip(char *buf, size_t len);
bool wifi_driver_ap_running(void);
void wifi_driver_get_ap_ssid(char *buf, size_t len);
