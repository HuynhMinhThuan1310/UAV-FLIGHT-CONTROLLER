/**
 * @file web_server.c
 * @brief WiFi SoftAP and HTTP Server implementation for Ground Station
 */

#include "web_server.h"
#include "web_dashboard.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "WEB_SERVER";

#define WIFI_SSID      "UAV-GroundStation"
#define WIFI_PASS      "uavtelemetry"
#define WIFI_CHANNEL   6
#define MAX_STA_CONN   4

static httpd_handle_t server = NULL;
static SemaphoreHandle_t state_mutex = NULL;
static GroundStationState gs_state;

void ground_station_update_state(const GroundStationState *new_state)
{
    if (state_mutex != NULL && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        gs_state = *new_state;
        xSemaphoreGive(state_mutex);
    }
}

void ground_station_get_state(GroundStationState *out_state)
{
    if (state_mutex != NULL && xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *out_state = gs_state;
        xSemaphoreGive(state_mutex);
    }
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_telemetry_get_handler(httpd_req_t *req)
{
    GroundStationState state;
    ground_station_get_state(&state);

    char json[768];
    snprintf(json, sizeof(json),
        "{\"link_ok\":%s,\"pkts\":%lu,\"loss\":%lu,\"hz\":%.1f,\"link_pct\":%.0f,\"uptime_s\":%lu,"
        "\"armed\":%s,\"failsafe\":%s,\"fs_mode\":%u,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"hdg\":%.2f,\"mag_hdg\":%.2f,"
        "\"m1\":%u,\"m2\":%u,\"m3\":%u,\"m4\":%u,"
        "\"pid_r\":%.1f,\"pid_p\":%.1f,\"pid_y\":%.1f,"
        "\"roll_stick\":%.2f,\"pitch_stick\":%.2f,\"yaw_stick\":%.2f,\"thr\":%.2f,\"servo\":%u,"
        "\"loop_dt\":%u}",
        state.link_ok ? "true" : "false",
        (unsigned long)state.rx_packet_count,
        (unsigned long)state.packet_loss_count,
        state.packet_rate_hz,
        state.link_quality_pct,
        (unsigned long)(state.uptime_ms / 1000),
        state.armed ? "true" : "false",
        state.failsafe ? "true" : "false",
        state.failsafe_mode,
        state.roll_deg,
        state.pitch_deg,
        state.heading_deg,
        state.mag_heading_deg,
        state.motor_us[0],
        state.motor_us[1],
        state.motor_us[2],
        state.motor_us[3],
        state.pid_out[0],
        state.pid_out[1],
        state.pid_out[2],
        state.roll_stick,
        state.pitch_stick,
        state.yaw_stick,
        state.throttle_norm,
        state.servo_us,
        state.loop_dt_avg_us
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_api_telemetry = {
    .uri       = "/api/telemetry",
    .method    = HTTP_GET,
    .handler   = api_telemetry_get_handler,
    .user_ctx  = NULL
};

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting HTTP Server on port %d...", config.server_port);
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_api_telemetry);
        ESP_LOGI(TAG, "Web server started! Open http://192.168.4.1 in your browser.");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP Server: %s", esp_err_to_name(ret));
    return ret;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Thiet bi ket noi: MAC="MACSTR" AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Thiet bi ngat ket noi: MAC="MACSTR" AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

esp_err_t wifi_init_softap(void)
{
    state_mutex = xSemaphoreCreateMutex();
    memset(&gs_state, 0, sizeof(gs_state));
    gs_state.motor_us[0] = 1000;
    gs_state.motor_us[1] = 1000;
    gs_state.motor_us[2] = 1000;
    gs_state.motor_us[3] = 1000;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi SoftAP khoi tao thanh cong:");
    ESP_LOGI(TAG, "  SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "  Pass: %s", WIFI_PASS);
    ESP_LOGI(TAG, "  IP  : 192.168.4.1 (Kenh WiFi: %d)", WIFI_CHANNEL);

    return ESP_OK;
}
