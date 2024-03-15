/* Sleep Functionality

   This code is derived from $ENV{IDF_PATH}/examples/system/deep_sleep

   Emumba
*/

#include "sleep_mode_management.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "SleepModeManagement";

bool ext1_wakeup = 0;        // To enable the External interrupt wake up feature
bool timer_wakeup = 0;       // To enable Timer based wake up feature
bool touch_wakeup = 1;       // To enable Touch Sensor based wake up feature
bool temperature_wakeup = 0; // To enable Temperature Sensor based wake up feature

void esp_sleep_start(sleep_mode_t mode)
{
    if (mode == LIGHT)
    {
        ESP_LOGI(TAG, "Entering light sleep mode.\n");
        vTaskDelay(100);
        esp_light_sleep_start();
    }
    else if (mode == DEEP)
    {
        ESP_LOGI(TAG, "Entering deep sleep mode.\n");
        vTaskDelay(100);
        esp_deep_sleep_start();
    }
}

void sleep_wakeup_handler(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause())
    {
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGI(TAG, "Wake up from GPIO %d\n", pin);
        }
        else
        {
            ESP_LOGI(TAG, "Wake up from GPIO\n");
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        ESP_LOGI(TAG, "Wake up from timer. Time spent in sleep: %dms\n", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
    {
        ESP_LOGI(TAG, "Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
        break;
    }
    case ESP_SLEEP_WAKEUP_ULP:
    {
        ESP_LOGI(TAG, "Wake up from ULP\n");
        int16_t diff_high = (int16_t)ulp_data_read(3);
        int16_t diff_low = (int16_t)ulp_data_read(4);
        if (diff_high < 0)
        {
            ESP_LOGI(TAG, "High temperature alarm was triggered\n");
        }
        else if (diff_low < 0)
        {
            ESP_LOGI(TAG, "Low temperature alarm was triggered\n");
        }
        else
        {
            assert(false && "Temperature has stayed within limits, but got ULP wakeup\n");
        }
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a sleep reset\n");
    }
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        ESP_LOGI(TAG, "ULP did %d temperature measurements in %d ms\n", ulp_data_read(1), sleep_time_ms);
        ESP_LOGI(TAG, "Initial T=%d, latest T=%d\n", ulp_data_read(0), ulp_data_read(2));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

//*****************Enabling features functions starting*********************//
void enable_timer_wakeup(int wakeup_time_sec)
{
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
    timer_wakeup = 1; // Status of timer wakeup set to one
}

void enable_ext1_wakeup(void)
{
    const int ext_wakeup_pin_1 = 2;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = 4;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;
    ESP_LOGI(TAG, "Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    ext1_wakeup = 1;
}

void enable_touch_pad_wakeup(void)
{
    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    ESP_ERROR_CHECK(touch_pad_init());
    // If use touch pad wake up, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // In this case, the high reference voltage will be 2.4V - 1V = 1.4V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    //init RTC IO and mode for touch pad.
    touch_pad_config(TOUCH_PAD_NUM8, TOUCH_THRESH_NO_USE);
    touch_pad_config(TOUCH_PAD_NUM9, TOUCH_THRESH_NO_USE);
    calibrate_touch_pad(TOUCH_PAD_NUM8);
    calibrate_touch_pad(TOUCH_PAD_NUM9);
    ESP_LOGI(TAG, "Enabling touch pad wakeup\n");
    esp_sleep_enable_touchpad_wakeup();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    touch_wakeup = 1;
}

void enable_ulp_temperature_wakeup(void)
{
    ESP_LOGI(TAG, "Enabling ULP wakeup\n");
    esp_sleep_enable_ulp_wakeup();
    ESP_LOGI(TAG, "Starting ULP temperature monitoring\n");
    start_ulp_temperature_monitoring();
    temperature_wakeup = 1;
}

//*****************Enabling features functions ended ********************//

//*****************Disabling features functions starting ********************//
void disable_touch_pad_wakeup(void)
{
    ESP_LOGI(TAG, "Disabling touch pad wakeup\n");

    esp_err_t err_status = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TOUCHPAD);

    if (err_status == ESP_OK)
    {
        ESP_LOGI(TAG, "Disabling touch pad wakeup successful.\n");
        touch_wakeup = 0;
    }
    else
    {
        ESP_LOGI(TAG, "Disabling touch pad wakeup not successful.\n");
    }
}

void disable_timer_wakeup(void)
{
    ESP_LOGI(TAG, "Disabling timer wakeup.\n");
    esp_err_t err_status = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);

    if (err_status == ESP_OK)
    {
        ESP_LOGI(TAG, "Disabling timer wakeup successful.\n");
        timer_wakeup = 0;
    }
    else
    {
        ESP_LOGI(TAG, "Disabling timer wakeup not successful\n");
    }
}

void disable_ext1_wakeup(void)
{
    ESP_LOGI(TAG, "Disabling ext1 wakeup.\n");
    esp_err_t err_status = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);

    if (err_status == ESP_OK)
    {
        ESP_LOGI(TAG, "Disabling ext1 wakeup successful.\n");
        ext1_wakeup = 0;
    }
    else
    {
        ESP_LOGI(TAG, "Disabling ext1 wakeup not successful.\n");
    }
}

void disable_ulp_temperature_wakeup(void)
{
    ESP_LOGI(TAG, "Disabling ULP Program (i.e. temperature) wakeup.\n");

    esp_err_t err_status = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ULP);

    if (err_status == ESP_OK)
    {
        ESP_LOGI(TAG, "Disabling ULP Program (i.e. temperature) wakeup successful.\n");
        temperature_wakeup = 0;
    }
    else
    {
        ESP_LOGI(TAG, "Disabling ULP Program (i.e. temperature) wakeup not successful.\n");
    }
}
//*****************Disabling features functions ended ********************//

//*****************Helping functions started******************************//
static void calibrate_touch_pad(touch_pad_t pad)
{
    int avg = 0;
    const size_t calibration_count = 128;
    for (int i = 0; i < calibration_count; ++i)
    {
        uint16_t val;
        touch_pad_read(pad, &val);
        avg += val;
    }
    avg /= calibration_count;
    const int min_reading = 300;
    if (avg < min_reading)
    {
        ESP_LOGI(TAG, "Touch pad #%d average reading is too low: %d (expecting at least %d). "
                      "Not using for sleep wakeup.\n",
                 pad, avg, min_reading);
        touch_pad_config(pad, 0);
    }
    else
    {
        int threshold = avg - 100;
        ESP_LOGI(TAG, "Touch pad #%d average: %d, wakeup threshold set to %d.\n", pad, avg, threshold);
        touch_pad_config(pad, threshold);
    }
}

static void start_ulp_temperature_monitoring(void)
{
    /*
     * This ULP program monitors the on-chip temperature sensor and wakes the chip up when
     * the temperature goes outside of certain window.
     * When the program runs for the first time, it saves the temperature measurement,
     * it is considered initial temperature (T0).
     *
     * On each subsequent run, temperature measured and compared to T0.
     * If the measured value is higher than T0 + max_temp_diff or lower than T0 - max_temp_diff,
     * the chip is woken up from sleep.
     */

    /* Temperature difference threshold which causes wakeup
     * With settings here (TSENS_CLK_DIV=2, 8000 cycles),
     * TSENS measurement is done in units of 0.73 degrees Celsius.
     * Therefore, max_temp_diff below is equivalent to ~2.2 degrees Celsius.
     */
    const int16_t max_temp_diff = 3;

    // Number of measurements ULP should do per second
    const uint32_t measurements_per_sec = 5;
    // Allow TSENS to be controlled by the ULP
    SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 2, SENS_TSENS_CLK_DIV_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);

    // Clear the part of RTC_SLOW_MEM reserved for the ULP. Makes debugging easier.
    memset(RTC_SLOW_MEM, 0, CONFIG_ESP32_ULP_COPROC_RESERVE_MEM);

    // The first word of memory (at data offset) is used to store the initial temperature (T0)
    // Zero it out here, then ULP will update it on the first run.
    ulp_data_write(0, 0);
    // The second word is used to store measuremeXnt count, zero it out as well.
    ulp_data_write(1, 0);

    const ulp_insn_t program[] = {
        // load data offset into R2
        I_MOVI(R2, ULP_DATA_OFFSET),
        // load/increment/store measurement counter using R1
        I_LD(R1, R2, 1),
        I_ADDI(R1, R1, 1),
        I_ST(R1, R2, 1),
        // enable temperature sensor
        I_WR_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, SENS_FORCE_XPD_SAR_S + 1, 3),
        // do temperature measurement and store result in R3
        I_TSENS(R3, 8000),
        // disable temperature sensor
        I_WR_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, SENS_FORCE_XPD_SAR_S + 1, 0),
        // Save current measurement at offset+2
        I_ST(R3, R2, 2),
        // load initial value into R0
        I_LD(R0, R2, 0),
        // if threshold value >=1 (i.e. initialized), goto 1
        M_BGE(1, 1),
        // otherwise, save the current value as initial (T0)
        I_MOVR(R0, R3),
        I_ST(R0, R2, 0),
        M_LABEL(1),
        // check if the temperature is greater or equal (T0 + max_temp_diff)
        // uses R1 as scratch register, difference is saved at offset + 3
        I_ADDI(R1, R0, max_temp_diff - 1),
        I_SUBR(R1, R1, R3),
        I_ST(R1, R2, 3),
        M_BXF(2),
        // check if the temperature is less or equal (T0 - max_temp_diff)
        // uses R1 as scratch register, difference is saved at offset + 4
        I_SUBI(R1, R0, max_temp_diff - 1),
        I_SUBR(R1, R3, R1),
        I_ST(R1, R2, 4),
        M_BXF(2),
        // temperature is within (T0 - max_temp_diff; T0 + max_temp_diff)
        // stop ULP until the program timer starts it again
        I_HALT(),
        M_LABEL(2),
        // temperature is out of bounds
        // disable ULP program timer
        I_WR_REG_BIT(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN_S, 0),
        // initiate wakeup of the SoC
        I_WAKE(),
        // stop the ULP program
        I_HALT()};

    // Load ULP program into RTC_SLOW_MEM, at offset 0
    size_t size = sizeof(program) / sizeof(ulp_insn_t);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(0, program, &size));
    assert(size < ULP_DATA_OFFSET && "ULP_DATA_OFFSET needs to be greater or equal to the program size");

    // Set ULP wakeup period
    const uint32_t sleep_cycles = rtc_clk_slow_freq_get_hz() / measurements_per_sec;
    REG_WRITE(SENS_ULP_CP_SLEEP_CYC0_REG, sleep_cycles);

    // Start ULP
    ESP_ERROR_CHECK(ulp_run(0));
}
//*****************Helping functions ended***********************************//
