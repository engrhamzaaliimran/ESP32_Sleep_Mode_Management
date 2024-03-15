/*
* This example demostrate the wakeup of main processors based on bringing the GPIO1 or GPIO2 
* to high level. The ULP will keep on monitoring the GPIOs level during the sleep mode.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sleep_mode_management.h"
#include "GPIO_pins_example.h"

static const char *TAG = "GPIOPinsBasedExample";

void application_task()
{
    sleep_wakeup_handler();
    while (1)
    {
        ESP_LOGI(TAG, "Enabling GPIO based wakeup\n");
        enable_ext1_wakeup();
        ESP_LOGI(TAG, "Entering Sleep. Handler of wakeup is monitoring the status.\n");
        vTaskDelay(100);
        esp_sleep_start(LIGHT);                    // Changing the parameter to 'DEEP' will change the sleep type to deep.
        ESP_LOGW(TAG, "Continued Working.....\n"); // Warning logs are used to change colour for highlighting purposes.
        vTaskDelay(2000);
        disable_ext1_wakeup();
    }
}
