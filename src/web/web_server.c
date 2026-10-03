#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "clock.h"
#include "defines.h"
#include "display.h"
#include "wifi_driver.h"
#include "wifi_store.h"
#include "web_server.h"

static const char *TAG = "WebServer";

static httpd_handle_t server_handle;
static char setup_notice[160];
static bool setup_notice_is_error;
static bool provision_busy;

typedef struct {
    char ssid[33];
    char pass[65];
    bool allow_hidden;
} provision_job_t;

static provision_job_t provision_job;

#define FORM_RECV_MAX_TIMEOUTS 3

static void record_save_error(esp_err_t *result, esp_err_t err)
{
    if (*result == ESP_OK && err != ESP_OK) {
        *result = err;
    }
}

static esp_err_t receive_form_body(httpd_req_t *req, char *body, size_t body_len)
{
    int total = req->content_len;
    if (total <= 0 || (size_t)total >= body_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    int off = 0;
    int timeouts = 0;
    while (off < total) {
        int n = httpd_req_recv(req, body + off, total - off);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts >= FORM_RECV_MAX_TIMEOUTS) {
                return ESP_ERR_TIMEOUT;
            }
            continue;
        }
        if (n <= 0) {
            return ESP_FAIL;
        }
        off += n;
        timeouts = 0;
    }
    body[off] = '\0';
    return ESP_OK;
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static void url_decode(char *dst, size_t dst_len, const char *src, size_t src_len)
{
    size_t di = 0;
    for (size_t si = 0; si < src_len && di + 1 < dst_len; si++) {
        char c = src[si];
        if (c == '+') {
            dst[di++] = ' ';
        } else if (c == '%' && si + 2 < src_len) {
            int hi = hex_val(src[si + 1]);
            int lo = hex_val(src[si + 2]);
            if (hi >= 0 && lo >= 0) {
                dst[di++] = (char)((hi << 4) | lo);
                si += 2;
            }
        } else {
            dst[di++] = c;
        }
    }
    dst[di] = '\0';
}

static bool form_value(const char *body, const char *key, char *out, size_t out_len)
{
    size_t key_len = strlen(key);
    const char *p = body;
    while (p && *p) {
        const char *amp = strchr(p, '&');
        size_t pair_len = amp ? (size_t)(amp - p) : strlen(p);
        const char *eq = memchr(p, '=', pair_len);
        if (eq) {
            size_t klen = (size_t)(eq - p);
            if (klen == key_len && memcmp(p, key, key_len) == 0) {
                url_decode(out, out_len, eq + 1, pair_len - klen - 1);
                return true;
            }
        }
        p = amp ? amp + 1 : NULL;
    }
    return false;
}

static void html_escape(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    if (!dst || dst_len == 0) {
        return;
    }
    for (; src && *src && di + 1 < dst_len; src++) {
        const char *rep = NULL;
        switch (*src) {
        case '&': rep = "&amp;"; break;
        case '<': rep = "&lt;"; break;
        case '>': rep = "&gt;"; break;
        case '"': rep = "&quot;"; break;
        case '\'': rep = "&#39;"; break;
        default:
            dst[di++] = *src;
            continue;
        }
        size_t n = strlen(rep);
        if (di + n >= dst_len) {
            break;
        }
        memcpy(dst + di, rep, n);
        di += n;
    }
    dst[di] = '\0';
}

static void send_html(httpd_req_t *req, const char *html, int len)
{
    size_t actual = strlen(html);
    if (len < 0 || (size_t)len > actual) {
        len = (int)actual;
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html, len);
}

static int setup_page(char *resp, size_t resp_len, const char *message, bool is_error, int refresh_seconds)
{
    char saved[WIFI_STORE_SSID_MAX + 1] = "";
    char pass[WIFI_STORE_PASS_MAX + 1];
    char saved_html[WIFI_STORE_SSID_MAX * 6 + 1] = "";
    if (!wifi_store_load(saved, sizeof(saved), pass, sizeof(pass))) {
        saved[0] = '\0';
    }
    html_escape(saved_html, sizeof(saved_html), saved);

    const bool show_message = message && message[0];
    return snprintf(resp, resp_len,
        "<!DOCTYPE html><html><head><title>Nixie Clock Wi-Fi</title>"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "%s"
        "<style>"
        "body{font-family:sans-serif;background:#111;color:#eee;max-width:28em;margin:40px auto;padding:0 16px;}"
        "input,button{font-size:1.1em;width:100%%;box-sizing:border-box;margin:6px 0 16px;padding:8px;}"
        "input[type=checkbox]{width:auto;margin-right:8px;}"
        "button{background:#4CAF50;color:#111;border:0;padding:12px;}"
        ".msg{color:#8af;margin-bottom:16px;}"
        ".err{color:#f44336;margin-bottom:16px;}"
        "label{color:#888;}"
        "</style></head><body>"
        "<h1>Wi-Fi setup</h1>"
        "%s%s%s"
        "<form method=\"POST\" action=\"/wifi\">"
        "<label>SSID</label><input name=\"ssid\" maxlength=\"32\" value=\"%s\" required>"
        "<label>Password</label><input name=\"password\" type=\"password\" maxlength=\"63\">"
        "<label><input type=\"checkbox\" name=\"hidden\" value=\"1\">Hidden network (not in scan)</label>"
        "<button type=\"submit\">Join</button>"
        "</form>"
        "</body></html>",
        refresh_seconds > 0 ? "<meta http-equiv=\"refresh\" content=\"2\">" : "",
        show_message ? (is_error ? "<p class=\"err\">" : "<p class=\"msg\">") : "",
        show_message ? message : "",
        show_message ? "</p>" : "",
        saved_html);
}

static bool ipv4_on_setup_ap(uint32_t addr)
{
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t info;
    if (!ap || esp_netif_get_ip_info(ap, &info) != ESP_OK || info.ip.addr == 0) {
        return false;
    }
    return (addr & info.netmask.addr) == (info.ip.addr & info.netmask.addr);
}

static bool request_on_setup_ap(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    struct sockaddr_storage addr;
    socklen_t addr_len = sizeof(addr);
    if (fd < 0 || getpeername(fd, (struct sockaddr *)&addr, &addr_len) != 0) {
        return false;
    }
    if (addr.ss_family == AF_INET) {
        return ipv4_on_setup_ap(((struct sockaddr_in *)&addr)->sin_addr.s_addr);
    }
    if (addr.ss_family == AF_INET6) {
        const struct in6_addr *ip6 = &((struct sockaddr_in6 *)&addr)->sin6_addr;
        if (!IN6_IS_ADDR_V4MAPPED(ip6)) {
            return false;
        }
        uint32_t v4;
        memcpy(&v4, &ip6->un.u32_addr[3], sizeof(v4));
        return ipv4_on_setup_ap(v4);
    }
    return false;
}

static int joined_page(char *resp, size_t resp_len)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);
    char tz_str[16];
    strftime(tz_str, sizeof(tz_str), "%Z", &timeinfo);

    char saved[WIFI_STORE_SSID_MAX + 1] = "";
    char saved_html[WIFI_STORE_SSID_MAX * 6 + 1] = "";
    char pass[WIFI_STORE_PASS_MAX + 1];
    wifi_store_load(saved, sizeof(saved), pass, sizeof(pass));
    html_escape(saved_html, sizeof(saved_html), saved[0] ? saved : "your home network");

    char ip[16] = "";
    wifi_driver_get_sta_ip(ip, sizeof(ip));

    return snprintf(resp, resp_len,
        "<!DOCTYPE html><html><head><title>Nixie Clock</title>"
        "<meta http-equiv=\"refresh\" content=\"5\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<style>"
        "body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:40px 16px;}"
        "h1{font-size:2em;margin-bottom:8px;}"
        ".label{color:#888;margin-bottom:0;}"
        "p{margin-top:4px;font-size:1.2em;}"
        "</style></head><body>"
        "<h1>Join network %s</h1>"
        "<p>Join %s to open the configuration page.</p>"
        "<p class=\"label\">Assigned IP-address</p><p>%s</p>"
        "<p class=\"label\">Time</p><p>%s</p>"
        "<p class=\"label\">Timezone</p><p>%s</p>"
        "<p class=\"label\">NTP Server</p><p>%s, %s</p>"
        "</body></html>",
        saved_html,
        saved_html,
        ip[0] ? ip : "unknown",
        time_str,
        tz_str,
        SNTP_TIME_SERVER,
        SNTP_TIME_SERVER_BACKUP);
}

static const char *CONFIG_CSS =
    "body{font-family:sans-serif;background:#111;color:#eee;max-width:28em;margin:24px auto;padding:0 16px;}"
    "nav{display:flex;flex-wrap:wrap;gap:8px 14px;margin:0 0 16px;}"
    "nav a{color:#8af;}"
    "input,select,button{width:100%;box-sizing:border-box;margin:8px 0 12px;padding:10px;font-size:1em;}"
    "input[type=checkbox]{width:auto;margin-right:8px;}"
    "button{background:#4CAF50;color:#111;border:0;}"
    ".mode{margin:0 0 28px;padding-bottom:12px;border-bottom:1px solid #333;}"
    ".mode h2{font-size:1.05em;font-weight:600;margin:8px 0 4px;}"
    ".msg{color:#8af;}.err{color:#f44336;}label{color:#aaa;display:block;}";

static const char *CONFIG_NAV =
    "<nav>"
    "<a href=\"/config?menu=display\">Display</a>"
    "<a href=\"/config?menu=bright\">Brightness</a>"
    "<a href=\"/config?menu=presence\">Presence</a>"
    "<a href=\"/config?menu=time\">Time</a>"
    "<a href=\"/config?menu=network\">Network</a>"
    "</nav>";

static void config_menu_name(httpd_req_t *req, char *menu, size_t len)
{
    menu[0] = '\0';
    if (req) {
        size_t qlen = httpd_req_get_url_query_len(req);
        if (qlen > 0 && qlen < 64) {
            char query[64];
            if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
                httpd_query_key_value(query, "menu", menu, len);
            }
        }
    }
    if (strcmp(menu, "bright") != 0 && strcmp(menu, "presence") != 0 &&
        strcmp(menu, "time") != 0 && strcmp(menu, "network") != 0) {
        strncpy(menu, "display", len - 1);
        menu[len - 1] = '\0';
    }
}

static int config_wrap(char *resp, size_t resp_len, const char *message, bool is_error, const char *body)
{
    bool show = message && message[0];
    return snprintf(resp, resp_len,
        "<!DOCTYPE html><html><head><title>Nixie clock</title>"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<style>%s</style></head><body><h1>Nixie clock</h1>%s%s%s%s%s%s%s</body></html>",
        CONFIG_CSS,
        show ? (is_error ? "<p id=\"status\" class=\"err\">" : "<p id=\"status\" class=\"msg\">") : "",
        show ? message : "",
        show ? "</p><script>setTimeout(function(){var s=document.getElementById('status');if(s)s.remove();}," : "",
        show ? (is_error ? "5000" : "2000") : "",
        show ? ");</script>" : "",
        CONFIG_NAV,
        body);
}

static int config_page(char *resp, size_t resp_len, const char *menu, const char *message, bool is_error)
{
    char body[4096];
    if (strcmp(menu, "bright") == 0) {
        snprintf(body, sizeof(body),
            "<form method=\"POST\" action=\"/config\" novalidate>"
            "<input type=\"hidden\" name=\"menu\" value=\"bright\">"
            "<label>Day brightness (0–100)</label>"
            "<input name=\"day\" type=\"number\" min=\"0\" max=\"100\" value=\"%u\" required>"
            "<label>Night brightness (0–100)</label>"
            "<input name=\"night\" type=\"number\" min=\"0\" max=\"100\" value=\"%u\" required>"
            "<label><input type=\"checkbox\" name=\"night_on\" id=\"night_on\" value=\"1\" %s>Night mode</label>"
            "<label>Night starts (hour)</label>"
            "<input name=\"nstart\" id=\"nstart\" type=\"number\" min=\"0\" max=\"23\" value=\"%u\"%s>"
            "<label>Night ends (hour)</label>"
            "<input name=\"nend\" id=\"nend\" type=\"number\" min=\"0\" max=\"23\" value=\"%u\"%s>"
            "<label>Transition (s)</label>"
            "<input name=\"trans\" type=\"number\" min=\"1\" max=\"60\" value=\"%u\" required>"
            "<button name=\"action\" value=\"trans_test\">Test transition</button>"
            "<button name=\"action\" value=\"save\">Save settings</button></form>"
            "<p>Night runs from the start hour until the end hour, and may cross midnight.</p>"
            "<script>function syncNight(){var off=!night_on.checked;nstart.disabled=off;nend.disabled=off;}"
            "night_on.onchange=syncNight;syncNight();</script>",
            clock_get_brightness(),
            clock_get_night_brightness(),
            clock_get_night_enabled() ? "checked" : "",
            clock_get_night_start(),
            clock_get_night_enabled() ? " required" : " disabled",
            clock_get_night_end(),
            clock_get_night_enabled() ? " required" : " disabled",
            clock_get_transition_s());
    } else if (strcmp(menu, "presence") == 0) {
        snprintf(body, sizeof(body),
            "<form method=\"POST\" action=\"/config\" novalidate>"
            "<input type=\"hidden\" name=\"menu\" value=\"presence\">"
            "<label><input type=\"checkbox\" name=\"presence\" id=\"presence\" value=\"1\" %s>Presence sensor</label>"
            "<label>Turn off after (minutes)</label>"
            "<input name=\"idle\" id=\"idle\" type=\"number\" min=\"1\" max=\"240\" value=\"%u\"%s>"
            "<button name=\"action\" value=\"save\">Save settings</button></form>"
            "<p>With the sensor on, the tubes turn off after this many minutes with nobody nearby.</p>"
            "<script>function syncPresence(){idle.disabled=!presence.checked;}"
            "presence.onchange=syncPresence;syncPresence();</script>",
            clock_get_presence_enabled() ? "checked" : "",
            clock_get_idle_minutes(),
            clock_get_presence_enabled() ? " required" : " disabled");
    } else if (strcmp(menu, "time") == 0) {
        char options[2200];
        size_t used = 0;
        int selected = clock_get_zone_index();
        if (selected < 0) {
            selected = 3;
        }
        for (int i = 0; i < clock_zone_count() && used < sizeof(options); i++) {
            used += snprintf(options + used, sizeof(options) - used,
                "<option value=\"%s\" data-dst=\"%d\"%s>%s</option>",
                clock_zone_id(i),
                clock_zone_has_dst(i) ? 1 : 0,
                i == selected ? " selected" : "",
                clock_zone_label(i));
        }
        char ntp1_html[256];
        char ntp2_html[256];
        char now[40];
        html_escape(ntp1_html, sizeof(ntp1_html), clock_get_ntp_primary());
        html_escape(ntp2_html, sizeof(ntp2_html), clock_get_ntp_backup());
        clock_format_now(now, sizeof(now));
        snprintf(body, sizeof(body),
            "<p>Clock time: %s</p>"
            "<form method=\"POST\" action=\"/config\" novalidate>"
            "<input type=\"hidden\" name=\"menu\" value=\"time\">"
            "<label>Timezone</label><select name=\"zone\" id=\"zone\">%s</select>"
            "<label><input type=\"checkbox\" name=\"dst\" id=\"dst\" value=\"1\" %s>Daylight saving</label>"
            "<button name=\"action\" value=\"save\">Save settings</button></form>"
            "<p>NTP server: %s</p><p>Backup NTP server: %s</p>"
            "<script>function syncTz(){var o=zone.selectedOptions[0];"
            "dst.disabled=o.dataset.dst=='0';}"
            "zone.onchange=syncTz;syncTz();</script>",
            now,
            options,
            (clock_get_dst() && clock_zone_has_dst(selected)) ? "checked" : "",
            ntp1_html,
            ntp2_html);
    } else if (strcmp(menu, "network") == 0) {
        char ip[16] = "";
        char ssid[WIFI_STORE_SSID_MAX + 1] = "";
        char pass[WIFI_STORE_PASS_MAX + 1];
        char ssid_html[WIFI_STORE_SSID_MAX * 6 + 1];
        uint8_t mac[6] = {0};
        int64_t uptime_s = esp_timer_get_time() / 1000000;
        wifi_driver_get_sta_ip(ip, sizeof(ip));
        if (!wifi_store_load(ssid, sizeof(ssid), pass, sizeof(pass))) {
            ssid[0] = '\0';
        }
        html_escape(ssid_html, sizeof(ssid_html), ssid[0] ? ssid : "none");
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        const esp_app_desc_t *app = esp_app_get_description();
        snprintf(body, sizeof(body),
            "<p>Connected network: %s</p>"
            "<p>Assigned IP: %s</p>"
            "<p>Firmware: %s</p>"
            "<p>MAC: %02x:%02x:%02x:%02x:%02x:%02x</p>"
            "<p>Uptime: %lld h %lld min</p>"
            "<form method=\"POST\" action=\"/config\" novalidate>"
            "<input type=\"hidden\" name=\"menu\" value=\"network\">"
            "<button name=\"action\" value=\"forget\">Forget current network</button></form>",
            ssid_html,
            ip[0] ? ip : "unknown",
            app ? app->version : "unknown",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
            (long long)(uptime_s / 3600),
            (long long)((uptime_s % 3600) / 60));
    } else {
        display_poison_cfg_t poison;
        display_random_cfg_t random_cfg;
        display_get_poison(&poison);
        display_get_random(&random_cfg);
        long poison_run_shown = poison.run_duration_ms < 0 ? 1
            : (poison.run_duration_ms + 500) / 1000;
        long random_run_shown = random_cfg.run_duration_ms < 0 ? 60
            : (random_cfg.run_duration_ms + 500) / 1000;
        snprintf(body, sizeof(body),
            "<form method=\"POST\" action=\"/config\" novalidate>"
            "<input type=\"hidden\" name=\"menu\" value=\"display\">"
            "<section class=\"mode\">"
            "<h2>Clock</h2>"
            "<label><input type=\"checkbox\" name=\"fade_on\" id=\"fade_on\" value=\"1\" %s>Fade effect</label>"
            "<label>Fade (ms)</label>"
            "<input name=\"fade\" id=\"fade\" type=\"number\" min=\"1\" max=\"5000\" value=\"%ld\"%s>"
            "<button name=\"action\" value=\"fade_test\">Test fade</button>"
            "<button name=\"action\" value=\"clock\">Show clock</button>"
            "</section>"
            "<section class=\"mode\">"
            "<h2>Cathode poisoning prevention routine</h2>"
            "<label>Digit hold (ms)</label>"
            "<input name=\"p_digit\" type=\"number\" min=\"1\" max=\"1000\" value=\"%ld\" required>"
            "<label>Offset</label>"
            "<input name=\"p_off\" type=\"number\" min=\"0\" max=\"12\" value=\"%u\" required>"
            "<label>Run time (s)</label>"
            "<input name=\"p_run\" id=\"p_run\" type=\"number\" min=\"0\" value=\"%ld\"%s>"
            "<label><input type=\"checkbox\" name=\"p_inf\" id=\"p_inf\" value=\"1\" %s>Run infinitely</label>"
            "<label><input type=\"checkbox\" name=\"p_inv\" value=\"1\" %s>Counts down</label>"
            "<label>Repeat every (minutes)</label>"
            "<input name=\"roll\" type=\"number\" min=\"1\" max=\"1440\" value=\"%u\" required>"
            "<button name=\"action\" value=\"poison\">Run cathode poisoning prevention routine</button>"
            "</section>"
            "<section class=\"mode\">"
            "<h2>Random digits</h2>"
            "<label>Digit hold (ms)</label>"
            "<input name=\"r_digit\" type=\"number\" min=\"1\" max=\"5000\" value=\"%ld\" required>"
            "<label>Run time (s)</label>"
            "<input name=\"r_run\" id=\"r_run\" type=\"number\" min=\"0\" value=\"%ld\"%s>"
            "<label><input type=\"checkbox\" name=\"r_inf\" id=\"r_inf\" value=\"1\" %s>Run infinitely</label>"
            "<button name=\"action\" value=\"random\">Run random</button>"
            "</section>"
            "<button name=\"action\" value=\"save\">Save settings</button></form>"
            "<script>"
            "function syncRun(box,input){input.disabled=box.checked;}"
            "function bind(box,input){box.onchange=function(){syncRun(box,input);};syncRun(box,input);}"
            "bind(p_inf,p_run);bind(r_inf,r_run);"
            "function syncFade(){fade.disabled=!fade_on.checked;}"
            "fade_on.onchange=syncFade;syncFade();"
            "</script>",
            display_get_fade_enabled() ? "checked" : "",
            (long)display_get_fade_ms(),
            display_get_fade_enabled() ? " required" : " disabled",
            (long)poison.digit_duration_ms,
            poison.offset,
            poison_run_shown,
            poison.run_duration_ms < 0 ? " disabled" : " required",
            poison.run_duration_ms < 0 ? "checked" : "",
            poison.inverse_direction ? "checked" : "",
            clock_get_roll_minutes(),
            (long)random_cfg.digit_duration_ms,
            random_run_shown,
            random_cfg.run_duration_ms < 0 ? " disabled" : " required",
            random_cfg.run_duration_ms < 0 ? "checked" : "");
    }
    return config_wrap(resp, resp_len, message, is_error, body);
}

static void provision_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));

    char err[160];
    esp_err_t join = wifi_driver_provision(provision_job.ssid, provision_job.pass,
                                           provision_job.allow_hidden, err, sizeof(err));
    if (join == ESP_OK) {
        esp_err_t save = wifi_store_save(provision_job.ssid, provision_job.pass);
        if (save == ESP_OK) {
            setup_notice[0] = '\0';
            setup_notice_is_error = false;
            wifi_driver_provision_commit();
        } else {
            snprintf(setup_notice, sizeof(setup_notice),
                     "Joined, but saving the network failed (%s). Please try again.",
                     esp_err_to_name(save));
            setup_notice_is_error = true;
            wifi_driver_enter_setup_mode();
        }
    } else {
        strncpy(setup_notice, err[0] ? err : "Could not join that network.", sizeof(setup_notice) - 1);
        setup_notice_is_error = true;
    }
    setup_notice[sizeof(setup_notice) - 1] = '\0';
    provision_busy = false;
    vTaskDelete(NULL);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    char resp[6144];
    int len;
    if (provision_busy) {
        len = setup_page(resp, sizeof(resp), "Joining…", false, 2);
    } else if (setup_notice[0]) {
        len = setup_page(resp, sizeof(resp), setup_notice, setup_notice_is_error, 0);
    } else if (wifi_driver_sta_connected()) {
        char menu[16];
        config_menu_name(req, menu, sizeof(menu));
        len = request_on_setup_ap(req) ? joined_page(resp, sizeof(resp))
                                       : config_page(resp, sizeof(resp), menu, NULL, false);
    } else {
        len = setup_page(resp, sizeof(resp), NULL, false, 0);
    }
    send_html(req, resp, len);
    return ESP_OK;
}

static esp_err_t wifi_post_handler(httpd_req_t *req)
{
    bool on_setup_ap = request_on_setup_ap(req);
    if (wifi_driver_sta_connected() && !provision_busy && !on_setup_ap) {
        char resp[6144];
        int len = config_page(resp, sizeof(resp), "display", NULL, false);
        send_html(req, resp, len);
        return ESP_OK;
    }

    char body[512];
    esp_err_t receive = receive_form_body(req, body, sizeof(body));
    if (receive == ESP_ERR_INVALID_SIZE) {
        char resp[6144];
        int len = setup_page(resp, sizeof(resp), "Form was empty or too large.", true, 0);
        send_html(req, resp, len);
        return ESP_OK;
    }
    if (receive == ESP_ERR_TIMEOUT) {
        char resp[6144];
        int len = setup_page(resp, sizeof(resp), "Timed out while receiving the form.", true, 0);
        send_html(req, resp, len);
        return ESP_OK;
    }
    if (receive != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete form");
        return ESP_FAIL;
    }

    char ssid[80];
    char pass[80];
    char hidden[8];
    if (!form_value(body, "ssid", ssid, sizeof(ssid))) {
        ssid[0] = '\0';
    }
    if (!form_value(body, "password", pass, sizeof(pass))) {
        pass[0] = '\0';
    }
    bool allow_hidden = form_value(body, "hidden", hidden, sizeof(hidden));

    char err[160];
    esp_err_t form = wifi_driver_check_credentials(ssid, pass, err, sizeof(err));
    if (form != ESP_OK) {
        char resp[6144];
        int len = setup_page(resp, sizeof(resp), err[0] ? err : "Check the SSID and password.", true, 0);
        send_html(req, resp, len);
        return ESP_OK;
    }
    if (provision_busy) {
        char resp[6144];
        int len = setup_page(resp, sizeof(resp), "Joining…", false, 2);
        send_html(req, resp, len);
        return ESP_OK;
    }

    strncpy(provision_job.ssid, ssid, sizeof(provision_job.ssid) - 1);
    provision_job.ssid[sizeof(provision_job.ssid) - 1] = '\0';
    strncpy(provision_job.pass, pass, sizeof(provision_job.pass) - 1);
    provision_job.pass[sizeof(provision_job.pass) - 1] = '\0';
    provision_job.allow_hidden = allow_hidden;
    setup_notice[0] = '\0';
    setup_notice_is_error = false;
    provision_busy = true;
    if (xTaskCreate(provision_task, "wifi_provision", 6144, NULL, 5, NULL) != pdPASS) {
        provision_busy = false;
        char resp[6144];
        int len = setup_page(resp, sizeof(resp), "Could not start joining.", true, 0);
        send_html(req, resp, len);
        return ESP_OK;
    }

    char resp[6144];
    int len = setup_page(resp, sizeof(resp), "Joining…", false, 2);
    send_html(req, resp, len);
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    if (!wifi_driver_sta_connected() || request_on_setup_ap(req)) {
        char resp[6144];
        int len = request_on_setup_ap(req) ? joined_page(resp, sizeof(resp))
                                           : setup_page(resp, sizeof(resp), NULL, false, 0);
        send_html(req, resp, len);
        return ESP_OK;
    }

    char body[768];
    char menu[16] = "display";
    esp_err_t receive = receive_form_body(req, body, sizeof(body));
    if (receive == ESP_ERR_INVALID_SIZE) {
        display_request_save_result(false, DISPLAY_AFTER_NONE);
        char resp[6144];
        int len = config_page(resp, sizeof(resp), menu, "The form was empty or too large.", true);
        send_html(req, resp, len);
        return ESP_OK;
    }
    if (receive == ESP_ERR_TIMEOUT) {
        display_request_save_result(false, DISPLAY_AFTER_NONE);
        char resp[6144];
        int len = config_page(resp, sizeof(resp), menu,
                              "Timed out while receiving the form.", true);
        send_html(req, resp, len);
        return ESP_OK;
    }
    if (receive != ESP_OK) {
        display_request_save_result(false, DISPLAY_AFTER_NONE);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete form");
        return ESP_FAIL;
    }

    char action[16];
    if (!form_value(body, "menu", menu, sizeof(menu))) {
        strncpy(menu, "display", sizeof(menu) - 1);
    }
    if (!form_value(body, "action", action, sizeof(action))) {
        action[0] = '\0';
    }
    if (strcmp(menu, "bright") != 0 && strcmp(menu, "presence") != 0 &&
        strcmp(menu, "time") != 0 && strcmp(menu, "network") != 0) {
        strncpy(menu, "display", sizeof(menu) - 1);
        menu[sizeof(menu) - 1] = '\0';
    }

    if (strcmp(menu, "network") == 0 && strcmp(action, "forget") == 0) {
        char resp[6144];
        int len = setup_page(resp, sizeof(resp), NULL, false, 0);
        send_html(req, resp, len);
        vTaskDelay(pdMS_TO_TICKS(400));
        wifi_store_erase();
        wifi_driver_enter_setup_mode();
        return ESP_OK;
    }

    const char *error = NULL;
    const char *save_section = NULL;
    esp_err_t save_err = ESP_OK;
    char save_error[160];
    if (strcmp(menu, "bright") == 0) {
        char day[8];
        char night[8];
        char nstart[8];
        char nend[8];
        char night_on[8];
        char trans[8];
        if (!form_value(body, "day", day, sizeof(day))) day[0] = '\0';
        if (!form_value(body, "night", night, sizeof(night))) night[0] = '\0';
        if (!form_value(body, "nstart", nstart, sizeof(nstart))) nstart[0] = '\0';
        if (!form_value(body, "nend", nend, sizeof(nend))) nend[0] = '\0';
        if (!form_value(body, "trans", trans, sizeof(trans))) trans[0] = '\0';
        int day_v = atoi(day);
        int night_v = atoi(night);
        int start_v = atoi(nstart);
        int end_v = atoi(nend);
        int trans_v = atoi(trans);
        bool night_enabled_form = form_value(body, "night_on", night_on, sizeof(night_on));
        if (day[0] == '\0' || day_v < 0 || day_v > 100) {
            error = "Day brightness must be 0–100.";
        } else if (night[0] == '\0' || night_v < 0 || night_v > 100) {
            error = "Night brightness must be 0–100.";
        } else if (night_enabled_form && (nstart[0] == '\0' || start_v < 0 || start_v > 23)) {
            error = "Night start must be an hour from 0–23.";
        } else if (night_enabled_form && (nend[0] == '\0' || end_v < 0 || end_v > 23)) {
            error = "Night end must be an hour from 0–23.";
        } else if (trans[0] == '\0' || trans_v < 1 || trans_v > 60) {
            error = "Brightness transition must be 1–60 seconds.";
        } else {
            save_section = "Brightness";
            record_save_error(&save_err, clock_set_brightness((uint8_t)day_v));
            record_save_error(&save_err, clock_set_night_brightness((uint8_t)night_v));
            record_save_error(&save_err, clock_set_night_enabled(night_enabled_form));
            if (night_enabled_form) {
                record_save_error(&save_err,
                                  clock_set_night_hours((uint8_t)start_v, (uint8_t)end_v));
            }
            record_save_error(&save_err, clock_set_transition_s((uint16_t)trans_v));
            if (save_err == ESP_OK && strcmp(action, "trans_test") == 0) {
                clock_test_brightness_transition();
            }
        }
    } else if (strcmp(menu, "presence") == 0) {
        char presence[8];
        char idle[8];
        if (!form_value(body, "idle", idle, sizeof(idle))) idle[0] = '\0';
        int idle_v = atoi(idle);
        bool presence_on = form_value(body, "presence", presence, sizeof(presence));
        if (presence_on && (idle[0] == '\0' || idle_v < 1 || idle_v > 240)) {
            error = "Turn off after must be 1–240 minutes.";
        } else {
            save_section = "Presence";
            record_save_error(&save_err, clock_set_presence_enabled(presence_on));
            if (presence_on) {
                record_save_error(&save_err, clock_set_idle_minutes((uint16_t)idle_v));
            }
        }
    } else if (strcmp(menu, "time") == 0) {
        char zone[16];
        char dst[8];
        if (!form_value(body, "zone", zone, sizeof(zone))) zone[0] = '\0';
        bool use_dst = form_value(body, "dst", dst, sizeof(dst));
        int index = -1;
        for (int i = 0; i < clock_zone_count(); i++) {
            if (strcmp(zone, clock_zone_id(i)) == 0) {
                index = i;
                break;
            }
        }
        if (index < 0) {
            error = "Choose a timezone.";
        } else {
            save_section = "Time";
            record_save_error(&save_err, clock_set_zone(index, use_dst));
        }
    } else {
        char p_digit[12];
        char p_off[8];
        char p_run[16];
        char p_inv[8];
        char p_inf[8];
        char r_digit[12];
        char r_run[16];
        char r_inf[8];
        char fade[12];
        char fade_on_buf[8];
        char roll[8];
        if (!form_value(body, "p_digit", p_digit, sizeof(p_digit))) p_digit[0] = '\0';
        if (!form_value(body, "p_off", p_off, sizeof(p_off))) p_off[0] = '\0';
        if (!form_value(body, "p_run", p_run, sizeof(p_run))) p_run[0] = '\0';
        if (!form_value(body, "r_digit", r_digit, sizeof(r_digit))) r_digit[0] = '\0';
        if (!form_value(body, "r_run", r_run, sizeof(r_run))) r_run[0] = '\0';
        if (!form_value(body, "fade", fade, sizeof(fade))) fade[0] = '\0';
        if (!form_value(body, "roll", roll, sizeof(roll))) roll[0] = '\0';
        bool poison_infinite = form_value(body, "p_inf", p_inf, sizeof(p_inf));
        bool random_infinite = form_value(body, "r_inf", r_inf, sizeof(r_inf));
        bool fade_on = form_value(body, "fade_on", fade_on_buf, sizeof(fade_on_buf));
        int poison_digit = atoi(p_digit);
        int poison_offset = atoi(p_off);
        int poison_seconds = atoi(p_run);
        int random_digit = atoi(r_digit);
        int random_seconds = atoi(r_run);
        int poison_run = poison_infinite ? -1
            : (poison_seconds >= 0 && poison_seconds <= 2000000 ? poison_seconds * 1000 : 0);
        int random_run = random_infinite ? -1
            : (random_seconds >= 0 && random_seconds <= 2000000 ? random_seconds * 1000 : 0);
        int fade_ms = atoi(fade);
        int roll_min = atoi(roll);
        if (fade_on && (fade[0] == '\0' || fade_ms < 1 || fade_ms > 5000)) {
            error = "Fade must be 1–5000 ms.";
        } else if (p_digit[0] == '\0' || poison_digit < 1 || poison_digit > 1000) {
            error = "Cathode poisoning digit hold must be 1–1000 ms.";
        } else if (poison_offset < 0 || poison_offset > 12) {
            error = "Cathode poisoning offset must be 0–12.";
        } else if (!poison_infinite && (p_run[0] == '\0' || poison_seconds < 0 || poison_seconds > 2000000)) {
            error = "Cathode poisoning run time must be 0 seconds or more.";
        } else if (roll[0] == '\0' || roll_min < 1 || roll_min > 1440) {
            error = "Repeat every must be 1–1440 minutes.";
        } else if (r_digit[0] == '\0' || random_digit < 1 || random_digit > 5000) {
            error = "Random digit hold must be 1–5000 ms.";
        } else if (!random_infinite && (r_run[0] == '\0' || random_seconds < 0 || random_seconds > 2000000)) {
            error = "Random run time must be 0 seconds or more.";
        } else {
            display_poison_cfg_t poison = {
                .digit_duration_ms = poison_digit,
                .run_duration_ms = poison_run,
                .offset = (uint8_t)poison_offset,
                .inverse_direction = form_value(body, "p_inv", p_inv, sizeof(p_inv)),
            };
            display_random_cfg_t random_cfg = {
                .digit_duration_ms = random_digit,
                .run_duration_ms = random_run,
            };
            save_section = "Display";
            record_save_error(&save_err, display_set_poison(&poison));
            record_save_error(&save_err, display_set_random(&random_cfg));
            record_save_error(&save_err, display_set_fade_enabled(fade_on));
            if (fade_on) {
                record_save_error(&save_err, display_set_fade_ms(fade_ms));
            }
            record_save_error(&save_err, clock_set_roll_minutes((uint16_t)roll_min));
        }
    }

    if (!error && save_err != ESP_OK) {
        snprintf(save_error, sizeof(save_error), "%s settings could not be saved (%s).",
                 save_section ? save_section : "Configuration", esp_err_to_name(save_err));
        ESP_LOGE(TAG, "%s", save_error);
        error = save_error;
    }

    display_after_t after = DISPLAY_AFTER_NONE;
    if (!error && strcmp(menu, "display") == 0) {
        if (strcmp(action, "poison") == 0) {
            after = DISPLAY_AFTER_POISON;
        } else if (strcmp(action, "random") == 0) {
            after = DISPLAY_AFTER_RANDOM;
        } else if (strcmp(action, "clock") == 0) {
            after = DISPLAY_AFTER_SHOW;
        } else if (strcmp(action, "fade_test") == 0) {
            after = DISPLAY_AFTER_FADE_TEST;
        }
    }
    display_request_save_result(error == NULL, after);

    char resp[6144];
    int len = config_page(resp, sizeof(resp), menu, error ? error : "Saved.", error != NULL);
    send_html(req, resp, len);
    return ESP_OK;
}

static const httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
};

static const httpd_uri_t wifi_get_uri = {
    .uri = "/wifi",
    .method = HTTP_GET,
    .handler = root_get_handler,
};

static const httpd_uri_t wifi_uri = {
    .uri = "/wifi",
    .method = HTTP_POST,
    .handler = wifi_post_handler,
};

static const httpd_uri_t config_get_uri = {
    .uri = "/config",
    .method = HTTP_GET,
    .handler = root_get_handler,
};

static const httpd_uri_t config_uri = {
    .uri = "/config",
    .method = HTTP_POST,
    .handler = config_post_handler,
};

esp_err_t web_server_start(void)
{
    if (server_handle) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.stack_size = 32768;
    config.max_req_hdr_len = 2048;
    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);

    if (httpd_start(&server_handle, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        server_handle = NULL;
        return ESP_FAIL;
    }

    httpd_register_uri_handler(server_handle, &root_uri);
    httpd_register_uri_handler(server_handle, &wifi_get_uri);
    httpd_register_uri_handler(server_handle, &wifi_uri);
    httpd_register_uri_handler(server_handle, &config_get_uri);
    httpd_register_uri_handler(server_handle, &config_uri);
    return ESP_OK;
}
