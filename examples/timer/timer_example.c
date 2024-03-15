#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sleep_mode_management.h"
#include "timer_example.h"

static const char *TAG = "TemperatureSensorExample";

void application_task()
{
    sleep_wakeup_handler();
    while (1)
    {
        ESP_LOGI(TAG,"Enabling timer based wakeup\n");
        enable_timer_wakeup(5);
        ESP_LOGI(TAG,"Entering Sleep. Handler of wakeup is monitoring the status.\n");
        vTaskDelay(100);
        esp_sleep_start(LIGHT); // Changing the parameter to 'DEEP' will change the sleep type to deep.
        ESP_LOGW(TAG,"Continued Working.....\n"); // Warning logs are used to change colour for highlighting purposes.
        disable_timer_wakeup();

    }
}
