/**
* @file main.cpp
* @brief LD2450 sensor test
 *
 * @author Emanuele Dolis (emanuele.dolis@gmail.com)
 * @version GIT_VERSION: v1.1.3-2-g1ca6c6c-dirty
 * @date 2026-06-24
 * @submodules-start
 * @submodules-end
 */
#include <stdio.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ED_LD2450.h"

#define CONFIG_IDF_TARGET_ESP32C6
#include "ed_board.h"

static const char *TAG = "MAIN";

// Adjust UART pins for your ESP32-C6 wiring
#define RADAR_UART_NUM   UART_NUM_1
#define RADAR_TX_PIN     GPIO_NUM_4
#define RADAR_RX_PIN     GPIO_NUM_5

extern "C" void app_main(void)
{
    ED_LD2450 radar;

    ESP_LOGI(TAG, "Initializing radar on UART%d", RADAR_UART_NUM);
    esp_err_t err = radar.begin(RADAR_UART_NUM, RADAR_TX_PIN, RADAR_RX_PIN, 256000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // let module stabilise

    // ---- Configuration ----
    ESP_LOGI(TAG, "Enabling configuration...");
    if (!radar.enableConfig()) {
        ESP_LOGE(TAG, "Enable config failed – check wiring and pull‑up resistors");
        return;
    }

    if (radar.setMultiTarget()) {
        ESP_LOGI(TAG, "Multi-target mode set");
    } else {
        ESP_LOGW(TAG, "Set multi-target failed, continuing anyway");
    }

    std::string fw;
    if (radar.readFirmwareVersion(fw)) {
        ESP_LOGI(TAG, "Firmware version: %s", fw.c_str());
    } else {
        ESP_LOGW(TAG, "Firmware query failed");
    }

    uint16_t mode;
    if (radar.queryTrackingMode(mode)) {
        ESP_LOGI(TAG, "Tracking mode: %s", (mode == 0x0001) ? "Single" : "Multi");
    }

    if (!radar.endConfig()) {
        ESP_LOGW(TAG, "End config failed");
    }
    ESP_LOGI(TAG, "Configuration done – starting data reception");

    // ---- Print column header once ----
    ESP_LOGI(TAG, "%-7s %-7s %-7s | %-7s %-7s %-7s | %-7s %-7s %-7s | %-7s",
             "T0 X", "T0 Y", "T0 Spd", "T1 X", "T1 Y", "T1 Spd", "T2 X", "T2 Y", "T2 Spd", "Res");
    ESP_LOGI(TAG, "-------+-------+-------+-------+-------+-------+-------+-------+-------+-------");

    // ---- Main loop – sample every 2 seconds ----
    uint32_t last_print_ms = 0;
    const uint32_t sample_interval_ms = 2000;   // 2 seconds

    while (1) {
        // Radar update must be called frequently to drain UART buffer
        radar.update();

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - last_print_ms) >= sample_interval_ms) {
            last_print_ms = now_ms;

            // Process the latest available frame
            if (radar.available()) {
                uint8_t cnt = radar.getTargetCount();

                // Read all three targets (fill absent ones with zeros)
                LD2450_Target targets[3];
                for (int i = 0; i < 3; i++) {
                    if (i < cnt && radar.getTarget(i, targets[i])) {
                        // target already filled
                    } else {
                        targets[i].x = 0;
                        targets[i].y = 0;
                        targets[i].speed = 0;
                        targets[i].resolution = 0;
                    }
                }

                // Build single aligned line
                char line[128];
                snprintf(line, sizeof(line),
                         "%7d %7d %7d | %7d %7d %7d | %7d %7d %7d | %7d",
                         targets[0].x, targets[0].y, targets[0].speed,
                         targets[1].x, targets[1].y, targets[1].speed,
                         targets[2].x, targets[2].y, targets[2].speed,
                         (cnt > 0 ? targets[0].resolution : 0)
                );
                ESP_LOGI(TAG, "%s", line);
            }
        }

        // Small delay – keep it short so radar.update() is called often
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}