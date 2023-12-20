#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Gesture.h"

pag7660 sensor;
static const char *TAG = "PAJ7620-Gesture";

extern "C" void app_main(void) {
    const char *cursor_str[] = {
        NULL,
        "Tap",
        "Grab",
        "Pinch",
    };

    vTaskDelay(pdMS_TO_TICKS(700));
    if (!sensor.init()) {
        ESP_LOGE(TAG, "PAJ7620 I2C error - halting");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "PAJ7620 init: OK");
    ESP_LOGI(TAG, "Please input your gestures:");

    while (1) {        
        pag7660_gesture_t result;
        if (sensor.getResult(result)) {
            switch (result.type) {
            case 0:
                switch (result.cursor.type) {
                case 1:
                case 2:
                case 3:
                    if (result.cursor.select)
                        ESP_LOGI(TAG, "%s", cursor_str[result.cursor.type]);
                    break;
                default:
                    break;
                }
                break;
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
                ESP_LOGI(TAG, "%d%s", result.type, "-finger");
                break;
            case 6:
                ESP_LOGI(TAG, "%s%d", "Rotate Right", result.rotate);
                break;
            case 7:
                ESP_LOGI(TAG, "%s%d", "Rotate Left", result.rotate);
                break;
            case 8:
                ESP_LOGI(TAG, "Swipe Left");
                break;
            case 9:
                ESP_LOGI(TAG, "Swipe Right");
                break;
            case 19:
            case 20:
            case 21:
            case 22:
            case 23:
                ESP_LOGI(TAG, "%d%s", (result.type - 19 + 1), "-finger push");
                break;
            default:
                break;
            }
        }   

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
