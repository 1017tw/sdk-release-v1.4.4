#ifndef _COMMON_36XX_LED_H_
#define _COMMON_36XX_LED_H_

#include "i2c.h"

#if USE_RTOS
#include "FreeRTOS.h"
#include "timers.h"
#include "FreeRTOS_POSIX/pthread.h"
#endif

#include <stdint.h>

/**
 * @brief private data for 36xx led device
 * @note see led_driver.h
 */
typedef struct {
    i2c_device_number_t i2c_num;          /** i2c number */
    uint8_t i2c_addr;                     /** i2c address */
    uint8_t i2c_addr_width;               /** i2c address width */
    uint32_t i2c_clk;                     /** i2c clk */
    uint8_t power_pin;                    /** power pin number */
    uint8_t led_index;                    /** which led is bind to this 36xx device */
    int use_strobe;                       /** use sensor strobe or gpio to trigger 36xx */
#if USE_RTOS
    TimerHandle_t timer;                  /** software timer which will be used to handle led timeout */
    pthread_mutex_t mutex;                /** internal mutex */
#endif
} common_led_t;

#endif
