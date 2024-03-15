#ifndef SLEEP_H
#define SLEEP_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"

#include "esp32/ulp.h"

#if SOC_TOUCH_SENSOR_NUM > 0
#include "soc/sens_periph.h"
#include "driver/touch_pad.h"
#endif

#define DEFAULT_WAKEUP_LEVEL ESP_GPIO_WAKEUP_GPIO_HIGH

static RTC_DATA_ATTR struct timeval sleep_enter_time;

typedef enum
{
    DEEP,  /* For entering deep sleep mode */
    LIGHT, /* For entering  light sleep mode*/
} sleep_mode_t;

/*
 * Offset (in 32-bit words) in RTC Slow memory where the data is placed
 * by the ULP coprocessor. It can be chosen to be any value greater or equal
 * to ULP program size and less than the CONFIG_ESP32_ULP_COPROC_RESERVE_MEM/4 - 6,
 * where 6 is the number of words used by the ULP coprocessor.
 */
#define ULP_DATA_OFFSET 36

_Static_assert(ULP_DATA_OFFSET < CONFIG_ESP32_ULP_COPROC_RESERVE_MEM / 4 - 6, "ULP_DATA_OFFSET is set too high, or CONFIG_ESP32_ULP_COPROC_RESERVE_MEM is not sufficient");

/**
 * @brief Start ULP temperature monitoring program
 *
 * This function loads a program into the RTC Slow memory and starts up the ULP.
 * The program monitors the on-chip temperature sensor and wakes up the SoC when
 * the temperature goes lower or higher than certain thresholds.
 */
static void start_ulp_temperature_monitoring(void);

/**
 * @brief Utility function which reads data written by ULP program
 *
 * @param offset offset from ULP_DATA_OFFSET in RTC Slow memory, in words
 * @return lower 16-bit part of the word writable by the ULP
 */
static inline uint16_t ulp_data_read(size_t offset)
{
    return RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] & 0xffff;
}

/**
 * @brief Utility function which writes data to be ready by ULP program
 *
 * @param offset offset from ULP_DATA_OFFSET in RTC Slow memory, in words
 * @param value lower 16-bit part of the word to be stored
 */
static inline void ulp_data_write(size_t offset, uint16_t value)
{
    RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] = value;
}

#define TOUCH_THRESH_NO_USE 0
static void calibrate_touch_pad(touch_pad_t pad);

/**
 * @brief Handles all the cases of awaking from sleep mode
 *
 * Handles the cases of awaking the processor based on 
 *  - Timer
 *  - Touch Sensor
 *  - Temperature Sensor
 *  - External interrupt
 */
void sleep_wakeup_handler(void);

/**
 * @brief Utility function which dose the average of values of touch sensor for robust performance
 * 
 * @param touch_pad_t Touch pad channel value
 */
static void calibrate_touch_pad(touch_pad_t pad);

/**
 * @brief This function uses ULP to monitors the on-chip temperature
 * 
 * This ULP program monitors the on-chip temperature sensor and wakes the chip up when
 * the temperature goes outside of a certain window.
 * When the program runs for the first time, it saves the temperature measurement,
 * it is considered initial temperature (T0).
 */
static void start_ulp_temperature_monitoring(void);

/**
 * @brief Enable timer based wakeup feature of ULP
 *
 * @param wakeup_time_sec The value for timer wakeup in seconds
 * 
 * This function makes a call to esp_sleep_enable_timer_wakeup to enable the timer
 * along with that it set the global status variable.
 */
void enable_timer_wakeup(int wakeup_time_sec);
/**
 * @brief Enable external pins wakeup
 * 
 * This function uses the external wakeup feature of the RTC controller. 
 * It will work even if RTC peripherals are shut down during sleep.
 * This feature can monitor GPIO1 and GPIO2 pins which are in RTC IOs. 
 * Once any of the selected pins go into the HIGH state, the chip will be woken up.
 */
void enable_ext1_wakeup(void);
/**
 * @brief Enable wakeup by touch sensor.
 *
 * This function enables the touch sensor wake-up feature of the RTC controller. 
 * It also sets global touch sensor status variable
 */
void enable_touch_pad_wakeup(void);
/**
 * @brief Enable on chip temperature sensor based wakeup
 *
 * This ULP program monitors the on-chip temperature sensor and wakes the chip up when
 * the temperature goes outside of a certain window.
 * When the program runs for the first time, it saves the temperature measurement,
 * it is considered initial temperature (T0).
 *
 * On each subsequent run, the temperature is measured and compared to T0.
 * If the measured value is higher than T0 + max_temp_diff or lower than T0 - max_temp_diff,
 * the chip is woken up from deep sleep.
*/
void enable_ulp_temperature_wakeup(void);

/**
 * @brief Disable timer based wakeup feature of ULP
 *
 * This function disables the timer-based wakeup and resets (set to 0)
 * the global status variable of timer-based wakeup feature. 
 */
void disable_timer_wakeup(void);

/**
 * @brief Disable wakeup by touch sensor.
 *
 * This function disables the touch sensor based wakeup and resets (set to 0)
 * the global status variable of touch sensor based wakeup feature. 
 */
void disable_touch_pad_wakeup(void);

/**
 * @brief Disable wakeup by external pins wakeup .
 *
 * This function disables the external pins wakeup based wakeup and resets (set to 0)
 * the global status variable of pins wakeup based wakeup feature. 
 */
void disable_ext1_wakeup(void);

/**
 * @brief Disable wakeup by internal temperature sensor wakeup .
 *
 * This function disables the internal temperature sensor wakeup based wakeup and resets (set to 0)
 * the global status variable of internal temperature sensor wake-up feature. 
 */
void disable_ulp_temperature_wakeup(void);

/**
 * @brief Starts light sleep or deep sleep power saving modes.
 *
 * @param mode if set LIGHT triggers light mode, else if set DEEP triggers deep mode.
 * 
 * In light sleep mode, digital peripherals, most of the RAM, and CPUs are clock-gated,
 * and supply voltage is reduced. Upon exit from light sleep, peripherals and CPUs resume operation,
 * their internal state is preserved.
 */
void esp_sleep_start(sleep_mode_t mode);

#endif /* SLEEP_H */
