/*
* The chip's build-in temperature sensor is highly unreliable and is recommended to be never
* used for any application. Still the feature was there and this examples tries to show its 
* functionality.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sleep_mode_management.h"
#include "temperature_sensor_example.h"

static const char *TAG = "TemperatureSensorExample";

void application_task()
{
    sleep_wakeup_handler();
    while (1)
    {
        ESP_LOGI(TAG,"Enabling temperature sensor wakeup\n");
        enable_ulp_temperature_wakeup();
        ESP_LOGI(TAG,"Entering Sleep. Handler of wakeup is monitoring the status.\n");
        vTaskDelay(100);
        esp_sleep_start(LIGHT); // Changing the parameter to 'DEEP' will change the sleep type to deep.
        ESP_LOGW(TAG,"Continued Working.....\n"); // Warning logs are used to change colour for highlighting purposes.
        disable_ulp_temperature_wakeup();

    }
}