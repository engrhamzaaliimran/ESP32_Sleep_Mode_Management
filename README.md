# Component Sleep Mode Management
This component provides the API calls for handling the sleep mode management-related work of main processors of ESP32 based boards. The ULP will be monitoring certain features and will generate the wake-up call.

## Difference in light and deep sleep modes
ESP32 is capable of light sleep and deep sleep power saving modes.

In light sleep mode, digital peripherals, most of the RAM, and CPUs are clock-gated, and supply voltage is reduced. Upon exit from light sleep, peripherals and CPUs resume operation, their internal state is preserved.

In deep sleep mode, CPUs, most of the RAM, and all the digital peripherals which are clocked from APB_CLK are powered off. The only parts of the chip which can still be powered on are:

- RTC controller
- RTC peripherals
- ULP coprocessor
- RTC fast memory
- RTC slow memory

## Features
The cases of awaking the processor from sleep mode handle by this component are following 
- Timer
- Touch Sensor
- Temperature Sensor
- External interrupt

The examples of each case are provided in the example folder. 

## API Calls
- void sleep_wakeup_handler(void);

  This API Handles the cases of awaking the processor based on Timer, Touch Sensor, Temperature Sensor, External interrupt. This API call needs to be made before enabling any feature for wakeup. This call is needed to be made just once. The recommended location for calling it is before entering the while loop of the task.

- void enable_timer_wakeup(void);

  Enable timer based wakeup feature of ULP. The default timer time is 5 seconds.

- void enable_ext1_wakeup(void);

  This function uses the external wakeup feature of the RTC controller. It will work even if RTC peripherals are shut down during sleep.This feature can monitor GPIO1 and GPIO2 pins which are in RTC IOs. Once any of the selected pins go into the HIGH state, the chip will be woken up.

- void enable_touch_pad_wakeup(void);

 This function enables the touch sensor wake-up feature of the RTC controller.

- void enable_ulp_temperature_wakeup(void);

  This ULP program monitors the on-chip temperature sensor and wakes the chip up when the temperature goes outside of a certain window. When the program runs for the first time, it saves the temperature measurement, it is considered initial temperature (T0). On each subsequent run, the temperature is measured and compared to T0. If the measured value is higher than T0 + max_temp_diff or lower than T0 - max_temp_diff, the chip is woken up from deep sleep.

**Warning**

The Temperature sensor built-in ESP32 chip is highly unreliable and is not recommended for any professional application.

- void disable_timer_wakeup(void);
  
  This function disables the timer-based wakeup and resets (set to 0) the global status variable of timer-based wakeup feature.

- void disable_touch_pad_wakeup(void);

  This function disables the touch sensor based wakeup and resets (set to 0) the global status variable of touch sensor based wakeup feature.

- void disable_ext1_wakeup(void);

  This function disables the external pins wakeup based wakeup and resets (set to 0) the global status variable of pins wakeup based wakeup feature. 

- void disable_ulp_temperature_wakeup(void);

  This function disables the internal temperature sensor wakeup based wakeup and resets (set to 0) the global status variable of internal temperature sensor wake-up feature. 

- void esp_sleep_start(sleep_mode_t mode);

  In light sleep mode, digital peripherals, most of the RAM, and CPUs are clock-gated, and supply voltage is reduced. Upon exit from light sleep, peripherals and CPUs resume operation, their internal state is preserved.