#include <stdio.h>
#include <string.h>

#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "mdns.h"

#include "defines.h"
#include "wifi_driver.h"
#include "wifi_store.h"

static const char *TAG = "WiFi";

#define PROBE_OK_BIT   BIT0
#define PROBE_FAIL_BIT BIT1
#define PROBE_TIMEOUT_MS 20000

static SemaphoreHandle_t wifi_lock;
static EventGroupHandle_t probe_events;
static wifi_got_ip_cb_t got_ip_cb;
static bool home_joined;
static bool wifi_started;
static TimerHandle_t ap_idle_timer;
static wifi_mode_t wifi_mode;
static bool want_sta;
static bool sta_connected;
static bool ap_running;
static bool probing;
static bool probe_armed;
static bool joined_once;
static int wifi_retry_num;
static uint8_t probe_reason;
static char sta_ssid[33];
static char ap_ssid[32];
static uint8_t ap_channel = 1;

static void wifi_lock_take(void)
{
    if (wifi_lock) {
        xSemaphoreTakeRecursive(wifi_lock, portMAX_DELAY);
    }
}

static void wifi_lock_give(void)
{
    if (wifi_lock) {
        xSemaphoreGiveRecursive(wifi_lock);
    }
}

static void make_ap_ssid(void)
{
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK) {
        memset(mac, 0, sizeof(mac));
    }
    snprintf(ap_ssid, sizeof(ap_ssid), "NixieClock-%02X%02X", mac[4], mac[5]);
}

static bool auth_is_psk(wifi_auth_mode_t mode)
{
    return mode == WIFI_AUTH_WPA_PSK
        || mode == WIFI_AUTH_WPA2_PSK
        || mode == WIFI_AUTH_WPA_WPA2_PSK
        || mode == WIFI_AUTH_WPA3_PSK
        || mode == WIFI_AUTH_WPA2_WPA3_PSK;
}

static void apply_sta_config(const char *ssid, const char *password, wifi_auth_mode_t authmode)
{
    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    if (password) {
        strncpy((char *)cfg.sta.password, password, sizeof(cfg.sta.password) - 1);
    }
    cfg.sta.threshold.authmode = authmode;
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
}

static void set_err(char *err_msg, size_t err_msg_len, const char *text)
{
    if (err_msg && err_msg_len > 0) {
        strncpy(err_msg, text, err_msg_len - 1);
        err_msg[err_msg_len - 1] = '\0';
    }
}

static bool password_rejected(uint8_t reason)
{
    return reason == WIFI_REASON_AUTH_FAIL
        || reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT
        || reason == WIFI_REASON_HANDSHAKE_TIMEOUT
        || reason == WIFI_REASON_802_1X_AUTH_FAILED
        || reason == WIFI_REASON_MIC_FAILURE;
}

static bool abandoning;
static bool sta_retry_pending;

static void abandon_task(void *arg)
{
    (void)arg;
    wifi_store_erase();
    ESP_LOGW(TAG, "Join failed; stored Wi-Fi credentials cleared");
    wifi_driver_enter_setup_mode();
    abandoning = false;
    vTaskDelete(NULL);
}

static void abandon_unjoined_credentials(void)
{
    if (abandoning) {
        return;
    }
    abandoning = true;
    want_sta = false;
    sta_ssid[0] = '\0';
    if (xTaskCreate(abandon_task, "wifi_abandon", 3072, NULL, 5, NULL) != pdPASS) {
        abandoning = false;
        wifi_store_erase();
    }
}

static void sta_retry_task(void *arg)
{
    (void)arg;
    /* Let the shared radio finish moving onto the home AP channel. */
    vTaskDelay(pdMS_TO_TICKS(500));
    sta_retry_pending = false;
    if (want_sta && sta_ssid[0] && !sta_connected && !probing && !abandoning) {
        esp_wifi_connect();
    }
    vTaskDelete(NULL);
}

static void schedule_sta_retry(void)
{
    if (sta_retry_pending) {
        return;
    }
    sta_retry_pending = true;
    if (xTaskCreate(sta_retry_task, "sta_retry", 3072, NULL, 5, NULL) != pdPASS) {
        sta_retry_pending = false;
        esp_wifi_connect();
    }
}
static const char *reason_text(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:
        return "Network not found.";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_AUTH_EXPIRE:
        return "Wrong password, or the network rejected the credentials.";
    default:
        return "Could not join that network.";
    }
}

static void apply_ap_config(void)
{
    if (ap_ssid[0] == '\0') {
        make_ap_ssid();
    }

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.ap.ssid, ap_ssid, sizeof(cfg.ap.ssid));
    cfg.ap.ssid_len = strlen(ap_ssid);
    cfg.ap.channel = ap_channel ? ap_channel : 1;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_OPEN;
    /* Beacons to send before a channel change so associated phones can follow. */
    cfg.ap.csa_count = 10;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
}

static void ensure_wifi_started(wifi_mode_t mode)
{
    if (!wifi_started) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_started = true;
        wifi_mode = mode;
        return;
    }
    if (wifi_mode == mode) {
        return;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    wifi_mode = mode;
}

static void ap_stop_task(void *arg)
{
    (void)arg;
    wifi_lock_take();
    wifi_sta_list_t clients = {0};
    if (home_joined && ap_running &&
        !(esp_wifi_ap_get_sta_list(&clients) == ESP_OK && clients.num > 0)) {
        ap_running = false;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        wifi_mode = WIFI_MODE_STA;
        ESP_LOGI(TAG, "Setup AP stopped after clients left");
    }
    wifi_lock_give();
    vTaskDelete(NULL);
}

static void ap_idle_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    if (!home_joined || !ap_running) {
        return;
    }
    xTaskCreate(ap_stop_task, "ap_stop", 4096, NULL, 5, NULL);
}

static void schedule_ap_shutdown(void)
{
    if (!home_joined || !ap_running) {
        return;
    }
    if (!ap_idle_timer) {
        ap_idle_timer = xTimerCreate("ap_idle", pdMS_TO_TICKS(10000), pdFALSE, NULL, ap_idle_timer_cb);
    }
    if (ap_idle_timer) {
        xTimerReset(ap_idle_timer, 0);
    }
}

static void cancel_ap_shutdown(void)
{
    if (ap_idle_timer) {
        xTimerStop(ap_idle_timer, 0);
    }
}

static void consider_ap_shutdown(void)
{
    wifi_sta_list_t clients = {0};
    if (esp_wifi_ap_get_sta_list(&clients) == ESP_OK && clients.num > 0) {
        cancel_ap_shutdown();
        return;
    }
    schedule_ap_shutdown();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (want_sta && sta_ssid[0]) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        sta_connected = false;
        if (probing && probe_armed) {
            const wifi_event_sta_disconnected_t *disc = event_data;
            probe_reason = disc ? disc->reason : 0;
            probe_armed = false;
            xEventGroupSetBits(probe_events, PROBE_FAIL_BIT);
            return;
        }
        if (!want_sta || probing) {
            return;
        }
        if (!joined_once) {
            const wifi_event_sta_disconnected_t *disc = event_data;
            uint8_t reason = disc ? disc->reason : 0;
            wifi_retry_num++;
            ESP_LOGW(TAG, "STA disconnected before getting an IP, retry %d reason %u (SSID:%s)",
                     wifi_retry_num, reason, sta_ssid);
            /* The first attempt often dies while the setup AP hops onto the home
             * channel. Retry that once. A repeated handshake failure is a bad password. */
            if (wifi_retry_num >= ESP_MAXIMUM_RETRY ||
                (password_rejected(reason) && wifi_retry_num > 1)) {
                abandon_unjoined_credentials();
                return;
            }
            schedule_sta_retry();
            return;
        }
        wifi_retry_num++;
        ESP_LOGW(TAG, "STA disconnected, retry %d (SSID:%s)", wifi_retry_num, sta_ssid);
        if (wifi_retry_num == ESP_MAXIMUM_RETRY && !ap_running) {
            ESP_LOGW(TAG, "STA join failed; starting setup AP");
            wifi_driver_start_ap();
        }
        if (want_sta) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        cancel_ap_shutdown();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (home_joined) {
            schedule_ap_shutdown();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP " IPSTR " (SSID:%s)", IP2STR(&event->ip_info.ip), sta_ssid);
        wifi_retry_num = 0;
        sta_connected = true;
        joined_once = true;
        if (probing && probe_armed) {
            probe_armed = false;
            xEventGroupSetBits(probe_events, PROBE_OK_BIT);
            return;
        }
        home_joined = true;
        consider_ap_shutdown();
        if (got_ip_cb) {
            got_ip_cb();
        }
    }
}

void wifi_driver_init(void)
{
    wifi_lock = xSemaphoreCreateRecursiveMutex();
    probe_events = xEventGroupCreate();
    configASSERT(wifi_lock);
    configASSERT(probe_events);

    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta, "nixie"));
    ESP_ERROR_CHECK(esp_netif_set_hostname(ap, "nixie"));

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("nixie"));
    ESP_ERROR_CHECK(mdns_instance_name_set("Nixie Clock"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    make_ap_ssid();
    ESP_LOGI(TAG, "Wi-Fi driver initialized (http://nixie.local/, setup AP %s)", ap_ssid);
}

void wifi_driver_set_got_ip_cb(wifi_got_ip_cb_t cb)
{
    got_ip_cb = cb;
}

esp_err_t wifi_driver_start_sta(const char *ssid, const char *password)
{
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_lock_take();
    strncpy(sta_ssid, ssid, sizeof(sta_ssid) - 1);
    sta_ssid[sizeof(sta_ssid) - 1] = '\0';
    want_sta = true;
    sta_connected = false;
    wifi_retry_num = 0;
    ensure_wifi_started(ap_running ? WIFI_MODE_APSTA : WIFI_MODE_STA);
    apply_sta_config(ssid, password, (password && password[0]) ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN);
    esp_err_t err = esp_wifi_connect();
    wifi_lock_give();

    ESP_LOGI(TAG, "STA connect started (SSID:%s)", sta_ssid);
    return err;
}

esp_err_t wifi_driver_check_credentials(const char *ssid, const char *password,
                                        char *err_msg, size_t err_msg_len)
{
    size_t ssid_len = ssid ? strlen(ssid) : 0;
    size_t pass_len = password ? strlen(password) : 0;

    if (ssid_len == 0 || ssid_len > 32) {
        set_err(err_msg, err_msg_len, "SSID must be 1–32 characters.");
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(ssid, ap_ssid) == 0) {
        set_err(err_msg, err_msg_len, "That is this clock's setup network, not your home Wi-Fi.");
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < pass_len; i++) {
        if (!isprint((unsigned char)password[i])) {
            set_err(err_msg, err_msg_len, "Password must be printable characters.");
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (pass_len > 0 && (pass_len < 8 || pass_len > 63)) {
        set_err(err_msg, err_msg_len, "Password must be 8–63 characters, or empty for an open network.");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t wifi_driver_provision(const char *ssid, const char *password, bool allow_hidden,
                                char *err_msg, size_t err_msg_len)
{
    if (err_msg && err_msg_len > 0) {
        err_msg[0] = '\0';
    }
    esp_err_t form = wifi_driver_check_credentials(ssid, password, err_msg, err_msg_len);
    if (form != ESP_OK) {
        return form;
    }

    const bool has_pass = password && password[0];
    wifi_auth_mode_t auth = has_pass ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;

    wifi_lock_take();
    ap_running = true;
    probing = true;
    probe_armed = false;
    want_sta = false;
    sta_connected = false;
    ensure_wifi_started(WIFI_MODE_APSTA);
    wifi_lock_give();

    wifi_scan_config_t scan_cfg = {
        .show_hidden = true,
    };
    esp_err_t scan_err = esp_wifi_scan_start(&scan_cfg, true);
    bool found = false;
    wifi_auth_mode_t found_auth = WIFI_AUTH_OPEN;
    uint8_t found_channel = 0;
    if (scan_err == ESP_OK) {
        uint16_t count = 16;
        wifi_ap_record_t records[16];
        if (esp_wifi_scan_get_ap_records(&count, records) == ESP_OK) {
            for (uint16_t i = 0; i < count; i++) {
                if (strncmp((const char *)records[i].ssid, ssid, sizeof(records[i].ssid)) == 0) {
                    found = true;
                    found_auth = records[i].authmode;
                    found_channel = records[i].primary;
                    break;
                }
            }
        }
    } else {
        ESP_LOGW(TAG, "Scan failed: %s", esp_err_to_name(scan_err));
    }

    if (!found && !allow_hidden) {
        probing = false;
        set_err(err_msg, err_msg_len,
                "Network not found. Check the name, or tick hidden network.");
        return ESP_ERR_NOT_FOUND;
    }
    if (found) {
        if (found_auth == WIFI_AUTH_OPEN && has_pass) {
            probing = false;
            set_err(err_msg, err_msg_len, "This network is open. Leave the password empty.");
            return ESP_ERR_INVALID_ARG;
        }
        if (found_auth != WIFI_AUTH_OPEN && !auth_is_psk(found_auth)) {
            probing = false;
            set_err(err_msg, err_msg_len, "This network's security is not supported.");
            return ESP_ERR_NOT_SUPPORTED;
        }
        if (auth_is_psk(found_auth) && !has_pass) {
            probing = false;
            set_err(err_msg, err_msg_len, "This network needs a password of 8–63 characters.");
            return ESP_ERR_INVALID_ARG;
        }
        auth = (found_auth == WIFI_AUTH_OPEN) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_PSK;
        if (found_channel >= 1 && found_channel <= 13) {
            /* Remember the home channel. Do not reconfigure the setup AP here:
             * set_config restarts it and deauthenticates the phone. esp_wifi_connect()
             * moves the shared radio and the AP announces the switch via csa_count. */
            ap_channel = found_channel;
        }
    }

    wifi_lock_take();
    strncpy(sta_ssid, ssid, sizeof(sta_ssid) - 1);
    sta_ssid[sizeof(sta_ssid) - 1] = '\0';
    want_sta = false;
    apply_sta_config(ssid, password, auth);
    xEventGroupClearBits(probe_events, PROBE_OK_BIT | PROBE_FAIL_BIT);
    probe_armed = true;
    esp_err_t conn = esp_wifi_connect();
    wifi_lock_give();
    if (conn != ESP_OK) {
        probe_armed = false;
        probing = false;
        set_err(err_msg, err_msg_len, "Could not start joining that network.");
        return conn;
    }

    EventBits_t bits = xEventGroupWaitBits(probe_events, PROBE_OK_BIT | PROBE_FAIL_BIT,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(PROBE_TIMEOUT_MS));
    probe_armed = false;

    if (bits & PROBE_OK_BIT && sta_connected) {
        probing = false;
        want_sta = true;
        joined_once = true;
        wifi_retry_num = 0;
        return ESP_OK;
    }

    want_sta = false;
    if (!(bits & PROBE_FAIL_BIT)) {
        esp_wifi_disconnect();
    }
    probing = false;
    if (bits & PROBE_FAIL_BIT) {
        set_err(err_msg, err_msg_len, reason_text(probe_reason));
        return ESP_FAIL;
    }
    set_err(err_msg, err_msg_len, "Timed out waiting for the network.");
    return ESP_ERR_TIMEOUT;
}

void wifi_driver_provision_commit(void)
{
    wifi_lock_take();
    probing = false;
    want_sta = true;
    home_joined = true;
    consider_ap_shutdown();
    wifi_lock_give();
    if (got_ip_cb) {
        got_ip_cb();
    }
}

esp_err_t wifi_driver_start_ap(void)
{
    wifi_lock_take();
    ap_running = true;
    ensure_wifi_started(WIFI_MODE_APSTA);
    apply_ap_config();
    wifi_lock_give();

    ESP_LOGI(TAG, "Setup AP '%s' at http://192.168.4.1/", ap_ssid);
    return ESP_OK;
}

esp_err_t wifi_driver_enter_setup_mode(void)
{
    wifi_lock_take();
    cancel_ap_shutdown();
    home_joined = false;
    joined_once = false;
    want_sta = false;
    sta_connected = false;
    probing = false;
    probe_armed = false;
    xEventGroupClearBits(probe_events, PROBE_OK_BIT | PROBE_FAIL_BIT);
    sta_ssid[0] = '\0';
    wifi_retry_num = 0;
    ap_running = true;
    ensure_wifi_started(WIFI_MODE_APSTA);
    apply_ap_config();
    esp_wifi_disconnect();
    wifi_lock_give();

    ESP_LOGI(TAG, "Setup AP '%s' at http://192.168.4.1/", ap_ssid);
    return ESP_OK;
}

bool wifi_driver_sta_connected(void)
{
    return sta_connected;
}

void wifi_driver_get_sta_ip(char *buf, size_t len)
{
    if (!buf || len == 0) {
        return;
    }
    buf[0] = '\0';
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (!netif || esp_netif_get_ip_info(netif, &ip) != ESP_OK || ip.ip.addr == 0) {
        return;
    }
    snprintf(buf, len, IPSTR, IP2STR(&ip.ip));
}

bool wifi_driver_ap_running(void)
{
    return ap_running;
}

void wifi_driver_get_ap_ssid(char *buf, size_t len)
{
    if (!buf || len == 0) {
        return;
    }
    strncpy(buf, ap_ssid, len - 1);
    buf[len - 1] = '\0';
}
