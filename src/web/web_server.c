#include <stdio.h>
#include <time.h>
#include <stdbool.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "defines.h"
#include "psu_driver.h"
#include "web_server.h"

static const char *TAG = "WebServer";

static httpd_handle_t server_handle;

static esp_err_t root_get_handler(httpd_req_t *req)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    bool psu_on = psu_driver_is_enabled();

    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

    char tz_str[16];
    strftime(tz_str, sizeof(tz_str), "%Z", &timeinfo);

    char resp[1024];
    int len = snprintf(resp, sizeof(resp),
        "<!DOCTYPE html><html><head><title>Nixie Clock</title>"
        "<meta http-equiv=\"refresh\" content=\"5\">"
        "<style>"
        "body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding-top:50px;}"
        "h1{font-size:2.5em;margin-bottom:0;}"
        ".label{color:#888;margin-bottom:0;}"
        "p{margin-top:4px;font-size:1.3em;}"
        ".on{color:#4CAF50;} .off{color:#f44336;}"
        "</style></head><body>"
        "<h1>%s</h1>"
        "<p class=\"label\">Timezone</p><p>%s</p>"
        "<p class=\"label\">NTP Server</p><p>%s</p>"
        "<p class=\"label\">HV PSU</p><p class=\"%s\">%s</p>"
        "</body></html>",
        time_str,
        tz_str,
        SNTP_TIME_SERVER,
        psu_on ? "on" : "off",
        psu_on ? "ON" : "OFF");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = root_get_handler,
    .user_ctx = NULL,
};

esp_err_t web_server_start(void)
{
    if (server_handle) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);

    if (httpd_start(&server_handle, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        server_handle = NULL;
        return ESP_FAIL;
    }

    httpd_register_uri_handler(server_handle, &root_uri);
    return ESP_OK;
}
