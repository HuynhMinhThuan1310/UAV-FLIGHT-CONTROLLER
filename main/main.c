/**
 * @file main.c
 * @brief UAV Ground Station for ESP32-C3
 *        - Nhan goi tin telemetry tu STM32 qua nRF24L01 (Kenh 80, 1Mbps, 100Hz)
 *        - Giai ma Att, RC, Status, Motors, Heading
 *        - Mo WiFi SoftAP & Web Server hien thi Dashboard thoi gian thuc tren trinh duyet
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "telemetry_protocol.h"
#include "nrf24_rx.h"
#include "web_server.h"

static const char *TAG = "GCS_MAIN";

static GroundStationState g_state;
static uint32_t rx_pkts_total = 0;
static uint32_t loss_pkts_total = 0;
static uint32_t rx_pkts_last_sec = 0;
static uint32_t loss_pkts_last_sec = 0;
static uint8_t  last_seq = 0;
static bool     last_seq_valid = false;
static int64_t  last_rx_time_us = 0;

static inline int16_t get_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Task thu nhan va giai ma goi tin nRF24L01 */
static void telemetry_rx_task(void *pvParameters)
{
    uint8_t packet[TEL_PACKET_SIZE];
    ESP_LOGI(TAG, "Telemetry RX Task started.");

    while (1) {
        /* Doc lien tuc cac goi tin co trong RX FIFO */
        while (nrf24_rx_read_packet(packet)) {
            if (!tel_packet_valid(packet)) {
                continue; /* Bo qua goi loi CRC */
            }

            int64_t now_us = esp_timer_get_time();
            last_rx_time_us = now_us;
            rx_pkts_total++;

            /* Tinh toan mat goi qua seq */
            uint8_t seq = packet[1];
            if (last_seq_valid) {
                uint8_t diff = (uint8_t)(seq - last_seq - 1);
                if (diff > 0 && diff < 128) {
                    loss_pkts_total += diff;
                }
            }
            last_seq = seq;
            last_seq_valid = true;

            uint8_t frame_type = packet[0];

            if (frame_type == TEL_FRAME_ATT) {
                /* 0xA1: ATTITUDE (50 Hz) */
                g_state.roll_deg     = get_i16(packet + 4) / 100.0f;
                g_state.pitch_deg    = get_i16(packet + 6) / 100.0f;
                g_state.heading_deg  = get_u16(packet + 8) / 100.0f;

                for (int i = 0; i < 3; i++) {
                    g_state.gyro_dps[i] = get_i16(packet + 10 + (2 * i)) / 10.0f;
                    g_state.pid_out[i]  = get_i16(packet + 16 + (2 * i)) / 10.0f;
                }

                for (int i = 0; i < 4; i++) {
                    g_state.motor_us[i] = get_u16(packet + 22 + (2 * i));
                }

                uint8_t flags_lo = packet[30];
                g_state.armed = (flags_lo & TEL_FLAG_ARMED) != 0;
                g_state.failsafe = (flags_lo & TEL_FLAG_FAILSAFE) != 0;

            } else if (frame_type == TEL_FRAME_RC) {
                /* 0xA2: RC STICK INPUTS (25 Hz) */
                uint16_t r_raw = get_u16(packet + 4);
                uint16_t p_raw = get_u16(packet + 6);
                uint16_t y_raw = get_u16(packet + 8);
                uint16_t t_raw = get_u16(packet + 10);

                /* Chuan hoa stick -1.0 .. +1.0 */
                g_state.roll_stick  = (r_raw - 1500) / 450.0f;
                g_state.pitch_stick = (p_raw - 1500) / 450.0f;
                g_state.yaw_stick   = (y_raw - 1500) / 450.0f;
                g_state.throttle_norm = (t_raw > 1000) ? ((t_raw - 1000) / 1000.0f) : 0.0f;

                g_state.servo_us = get_u16(packet + 14);

            } else if (frame_type == TEL_FRAME_STATUS) {
                /* 0xA3: FC HEALTH & STATUS (10 Hz) */
                g_state.uptime_ms      = get_u32(packet + 2);
                g_state.flags          = get_u16(packet + 6);
                g_state.failsafe_mode  = packet[8];
                g_state.loop_dt_avg_us = get_u16(packet + 16);

                g_state.armed    = (g_state.flags & TEL_FLAG_ARMED) != 0;
                g_state.failsafe = (g_state.flags & TEL_FLAG_FAILSAFE) != 0;

            } else if (frame_type == TEL_FRAME_TUNE) {
                /* 0xA4: COMPASS & TUNING (10 Hz) */
                g_state.mag_heading_deg = get_u16(packet + 12) / 100.0f;
            }

            g_state.rx_packet_count   = rx_pkts_total;
            g_state.packet_loss_count = loss_pkts_total;
            ground_station_update_state(&g_state);
        }

        /* Nghi 2ms nhan nhip tiep theo (khong chiem dung CPU) */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* Task tinh toan thong ke Hz, Link Quality va in Serial Console */
static void telemetry_stats_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t delta_rx   = rx_pkts_total - rx_pkts_last_sec;
        uint32_t delta_loss = loss_pkts_total - loss_pkts_last_sec;
        rx_pkts_last_sec    = rx_pkts_total;
        loss_pkts_last_sec  = loss_pkts_total;

        int64_t now_us = esp_timer_get_time();
        bool link_active = (last_rx_time_us > 0) && ((now_us - last_rx_time_us) < 600000); /* 600ms timeout */

        float hz = (float)delta_rx;
        float total_sent = (float)(delta_rx + delta_loss);
        float quality = (total_sent > 0) ? (100.0f * (float)delta_rx / total_sent) : (link_active ? 100.0f : 0.0f);

        g_state.link_ok          = link_active;
        g_state.packet_rate_hz   = hz;
        g_state.link_quality_pct = quality;
        ground_station_update_state(&g_state);

        if (link_active) {
            ESP_LOGI(TAG, "[UAV LINK OK] %4.1f Hz | Loss: %ld | Roll: %+5.1f° Pitch: %+5.1f° Hdg: %5.1f° | Motors: %4d %4d %4d %4d | ARM: %d",
                     hz,
                     (long)loss_pkts_total,
                     g_state.roll_deg,
                     g_state.pitch_deg,
                     g_state.heading_deg,
                     g_state.motor_us[0], g_state.motor_us[1], g_state.motor_us[2], g_state.motor_us[3],
                     g_state.armed ? 1 : 0);
        } else {
            ESP_LOGW(TAG, "[UAV LINK LOST] Cho tin hieu tu may bay tren Kenh 80 (2480 MHz)...");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "   ESP32-C3 UAV TELEMETRY GROUND STATION    ");
    ESP_LOGI(TAG, "=============================================");

    /* 1. Khoi tao NVS Flash (can cho WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Khoi tao WiFi SoftAP */
    ESP_ERROR_CHECK(wifi_init_softap());

    /* 3. Khoi dong Web Server Dashboard */
    ESP_ERROR_CHECK(web_server_start());

    /* 4. Khoi tao nRF24L01 Receiver */
    ret = nrf24_rx_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Loi khoi tao nRF24L01! Vui long kiem tra lai day noi SPI tren ESP32-C3.");
    }

    /* 5. Khoi tao cac FreeRTOS Task */
    xTaskCreate(telemetry_rx_task, "tel_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(telemetry_stats_task, "tel_stats_task", 3072, NULL, 1, NULL);

    ESP_LOGI(TAG, "Ground Station khoi dong hoan tat!");
    ESP_LOGI(TAG, "Ket noi vao WiFi 'UAV-GroundStation' (pass: uavtelemetry) va mo trinh duyet vao: http://192.168.4.1");
}
