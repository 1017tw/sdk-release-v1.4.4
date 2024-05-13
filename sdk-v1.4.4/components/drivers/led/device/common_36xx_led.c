#include "common_36xx_led.h"
#include "led_driver.h"
#include "gpio.h"
#include "sysctl.h"
#include "syslog.h"
#include "udevice.h"
#if USE_RTOS
#include <stdio.h>
#else
#include "aiva_sleep.h"
#endif
#include <math.h>
//#define USE_SENSOR_OV02B_STROBE
static led_channel_t  s_chn = CHANNEL_0;
typedef struct common_36xx_params {
    /** current_ma = current_level * current_step_ma + current_min_ma*/
    float current_step_ma;
    float current_min_ma;
    uint8_t chip_id;
    uint8_t device_id;
} common_36xx_params_t;

typedef enum {
    LM3644 = 0,
    AW36515,
    AW3644
} product_type_t;

static common_36xx_params_t supported_36xx_products[] = {
    [LM3644] = {
        .current_step_ma = 11.725f,
        .current_min_ma = 10.9f,
        .chip_id = 0x00,
        .device_id = 0x02,
    },
    [AW36515] = {
        .current_step_ma = 7.83f,
        .current_min_ma = 3.91f,
        .chip_id = 0x30,
        .device_id = 0x02,
    },
    [AW3644] = {
        .current_step_ma = 11.72f,
        .current_min_ma = 11.35f,
        .chip_id = 0x36,
        .device_id = 0x02,
    }
};
// default is LM3644
static product_type_t current_36xx_type = LM3644;

#define MAX_BRIGHTNESS (0x7F)
#define LED_BRIGHTNESS_INIT (0x20)

#define TAG                             "36XX"
#define LM3644_SLAVE_ADDR               (0x63 << 0)

#define CHIP_ID_REG                     (0x00) // only aw3644 aw36515 has this reg
#define ENABLE_REG                      (0X01)
#define IVFM_REG                        (0x02)
#define LED1_FLASH_REG                  (0x03)
#define LED2_FLASH_REG                  (0x04)
#define LED1_TORCH_REG                  (0x05)
#define LED2_TORCH_REG                  (0x06)
#define BOOST_REG                       (0x07)
#define TIMING_REG                      (0x08)
#define TEMP_REG                        (0x09)
#define FLAGS1_REG                      (0x0A)
#define FLAGS2_REG                      (0x0B)
#define DEVICE_ID_REG                   (0x0C)
#define LAST_FLASH_REG                  (0x0D)

enum common_36xx_time_out_val {
    LM3644_TIMEOUT_10MS   = 0,
    LM3644_TIMEOUT_20MS   = 1,
    LM3644_TIMEOUT_30MS   = 2,
    LM3644_TIMEOUT_40MS   = 3,
    LM3644_TIMEOUT_50MS   = 4,
    LM3644_TIMEOUT_60MS   = 5,
    LM3644_TIMEOUT_70MS   = 6,
    LM3644_TIMEOUT_80MS   = 7,
    LM3644_TIMEOUT_90MS   = 8,
    LM3644_TIMEOUT_100MS  = 9,
    LM3644_TIMEOUT_150MS  = 10,
    LM3644_TIMEOUT_200MS  = 11,
    LM3644_TIMEOUT_250MS  = 12,
    LM3644_TIMEOUT_300MS  = 13,
    LM3644_TIMEOUT_350MS  = 14,
    LM3644_TIMEOUT_400MS  = 15,
};

typedef struct _LM3644_CONFIG_T {
    uint8_t     reg_addr;                             // Register address
    uint8_t     data;                                 // Data
}LM3644_CONFIG_T;

/* 
   if only LED_2 is connected to lm3644, setting reg 0x1 to 0x27, LED_2 won't work;
   if LED_1 and LED_2 is connected to lm3644, setting reg 0x1 to 0x27, LED_1 and LED_2 works.
   */   
static LM3644_CONFIG_T common_36xx_init_settings[] = {
/*#ifdef USE_SL18*/
    /*// IR mode, 0x26, enable LED2,disable LED1.*/
    /*{0x1, 0x26},       // Reg 1  ()    */
/*#else*/
    /*// IR mode, 0x25, enable LED1,disable LED2.*/
    /*{0x1, 0x25},       // Reg 1  ()    */
/*#endif*/
    // IR mode, 0x27, disable LED2, LED1.
    {ENABLE_REG, 0x24},       // Reg 1  ()

    {IVFM_REG, 0x01},       // Reg 2  ()
    {LED1_FLASH_REG, LED_BRIGHTNESS_INIT},       // Reg 3  ()
    {LED2_FLASH_REG, LED_BRIGHTNESS_INIT},       // Reg 4  ()
    {LED1_TORCH_REG, 0x8F},       // Reg 5  ()
    {LED2_TORCH_REG, 0x0F},       // Reg 6  ()
    {BOOST_REG, 0x09},       // Reg 7  ()
    /*{TIMING_REG, 0x1F},       // Reg 8  ()*/
    {TIMING_REG, 0x13},       // Reg 8  ()  40ms
    /*{TIMING_REG, 0x14},       // Reg 8  ()  50ms*/
    /*{TIMING_REG, 0x11},       // Reg 8  ()  20ms*/
    {TEMP_REG, 0x08},       // Reg 9  ()
};

typedef struct _LED_STATUS {
    uint8_t     brightness_reg_addr;
    uint8_t     brightness;                             
} LED_STATUS;

static LED_STATUS led_status[] = {
    {LED1_FLASH_REG, LED_BRIGHTNESS_INIT},
    {LED2_FLASH_REG, LED_BRIGHTNESS_INIT}
};

static int common_36xx_write_reg(i2c_device_number_t dev, uint8_t i2c_addr, uint8_t reg, uint8_t data)
{
    int ret;
    uint8_t data_buf[2];

    data_buf[0] = reg;
    data_buf[1] = data;
    ret = i2c_send_data(dev, i2c_addr, data_buf, 2);
    CHECK_RET(ret);

    return ret;
}

static uint8_t common_36xx_read_reg(i2c_device_number_t dev, uint8_t i2c_addr, uint8_t reg_addr)
{
    int ret;
    uint8_t reg_val;

    ret = i2c_send_data(dev, i2c_addr, &reg_addr, 1);
    CHECK_RET(ret);

    ret = i2c_recv_data(dev, i2c_addr, 0, 0, &reg_val, 1);
    CHECK_RET(ret);

    return reg_val;
}

static int common_36xx_get_lock(led_dev_t *dev)
{
    int ret = 0;
#ifdef USE_RTOS
    if (!xPortIsInISR()) {
        ret = pthread_mutex_lock(&(((common_led_t*)dev->priv)->mutex));
    }
#endif
    return ret;
}

static int common_36xx_release_lock(led_dev_t *dev)
{
    int ret = 0;
#ifdef USE_RTOS
    if (!xPortIsInISR()) {
        ret = pthread_mutex_unlock(&(((common_led_t*)dev->priv)->mutex));
    }
#endif
    return ret;
}

static enum common_36xx_time_out_val timeout_ms2level(int timeout_ms)
{
    enum common_36xx_time_out_val timeout_level = LM3644_TIMEOUT_10MS;
    if (timeout_ms < 5)
    {
        timeout_level = LM3644_TIMEOUT_10MS;
    }
    else if (timeout_ms >= 5 && timeout_ms <= 100)
    {
        timeout_level = (enum common_36xx_time_out_val)((timeout_ms + 5) / 10 - 1);
    }
    else if (timeout_ms <= 400)
    {
        timeout_level = (enum common_36xx_time_out_val)((timeout_ms + 25) / 50 - 2 + LM3644_TIMEOUT_100MS);
    }
    else
    {
        timeout_level = LM3644_TIMEOUT_400MS;
    }

    return timeout_level;
}

static int common_36xx_set_flash_timeout(led_dev_t *dev, int timeout_level)//设置闪光灯的超时时间
{
    common_36xx_get_lock(dev);//代码获取一个锁，以确保在设置超时时间期间不会被其他操作打断

    common_led_t *_dev = (common_led_t *)dev->priv;
    i2c_device_number_t i2c_num = _dev->i2c_num;
    i2c_init(i2c_num, _dev->i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);

    int ret = common_36xx_write_reg(i2c_num,
            _dev->i2c_addr,
            TIMING_REG, 
            timeout_level);

    common_36xx_release_lock(dev);

    return ret;
}

// void common_36xx_test(void)
// {    
//     common_36xx_init(I2C_DEVICE_2, GPIO_PIN3, true);
//     common_36xx_set_flash_timeout(LM3644_TIMEOUT_10MS);

//     while (1)
//     {
//         common_36xx_power_off();        
//         common_36xx_power_on();
//         common_36xx_busy_delay(3000);
//     }
// }


#ifdef __cplusplus
extern "C" {
#endif


#if USE_RTOS
static void timeout_cb(TimerHandle_t timer)
{
    led_dev_t* led_dev = (led_dev_t*)pvTimerGetTimerID(timer);
    common_led_t* common_36xx_dev = (common_led_t*)led_dev->priv;
    gpio_pin_value_t pin_val = gpio_get_pin(common_36xx_dev->power_pin);
    if(pin_val == GPIO_PV_LOW)
    {
        // set power pin high to turn on led, and start timer to turn off led when timeout 
        gpio_set_pin(common_36xx_dev->power_pin, GPIO_PV_HIGH);
        LOGI("", "%d GPIO_PV_HIGH\n", (int)common_36xx_dev->power_pin);
        
        TimerHandle_t _timer = common_36xx_dev->timer;
        TickType_t period = xTimerGetPeriod(_timer);
        TickType_t expect_period = led_dev->timeout_ms[s_chn] / portTICK_PERIOD_MS;
        if (period != expect_period)
        {
            xTimerChangePeriod(_timer, expect_period, 0);
        }
        xTimerStart(_timer, 0);
    }
    else
    {
        // reset power pin to turn off led
       // LOGI("", "%d GPIO_PV_LOW\n", (int)common_36xx_dev->power_pin);
        gpio_set_pin(common_36xx_dev->power_pin, GPIO_PV_LOW);
    }
}
#endif

int common_36xx_init(led_dev_t *dev)
{
    common_led_t* _dev = (common_led_t*)dev->priv;
    uint8_t pin = _dev->power_pin;
    int timeout_ms = dev->timeout_ms;
 
    i2c_device_number_t i2c_num = _dev->i2c_num;

    common_36xx_get_lock(dev);

    i2c_init(i2c_num, _dev->i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);
    
    // read 0x00 to get product id
    uint8_t chip_id = common_36xx_read_reg(i2c_num, _dev->i2c_addr, CHIP_ID_REG);
#if 0 // LM3644 not tell us what will return when read 0x00, so we can not assume it not return 0x30 0r 0x36
    for (size_t i = 0; i < sizeof(supported_36xx_products) / sizeof(supported_36xx_products[0]); i++)
    {
        if (supported_36xx_products[i].chip_id == chip_id)
        {
            current_36xx_type = (product_type_t)i;
            break;
        }
    }
#else
    if (chip_id == supported_36xx_products[AW36515].chip_id)
    {
        current_36xx_type = AW36515;
    }
    else if (chip_id == supported_36xx_products[AW3644].chip_id)
    {
        current_36xx_type = AW3644;
    }
#endif

    int ret = 0;
    for (int i = 0; i < (int)AIVA_ARRAY_LEN(common_36xx_init_settings); i++)
    {
        ret = common_36xx_write_reg(i2c_num,
                _dev->i2c_addr,
                common_36xx_init_settings[i].reg_addr, 
                common_36xx_init_settings[i].data);
        if (ret != 0)
        {
            common_36xx_release_lock(dev);
            goto out;
        }
    }

    // enum common_36xx_time_out_val timeout_level = timeout_ms2level(timeout_ms);
    // common_36xx_write_reg(i2c_num,
    //     _dev->i2c_addr, 
    //     TIMING_REG, 
    //     timeout_level);

    common_36xx_release_lock(dev);

    sysctl_io_switch_t io_switch = IO_SWITCH_GPIO0 + pin;
    sysctl_set_io_switch(io_switch, 1);

    if (_dev->use_strobe)
    {
        gpio_set_drive_mode(pin, GPIO_DM_INPUT);
    }
    else
    {
        gpio_set_drive_mode(pin, GPIO_DM_OUTPUT);
#if USE_RTOS
        int pulse_in_ms = 2;
        const led_trig_param_t *trig = dev->trig_param;
        if (trig != NULL && trig->pulse_in_us >= 2000 && trig->pulse_in_us <= 10000)
        {
            pulse_in_ms = (trig->pulse_in_us / 1000);
        }
        char timer_name[20] = {0};
        snprintf(timer_name, sizeof(timer_name) / sizeof(timer_name[0]), "LM3644%d_LED_TIMER", (int)pin);
        // create software timer
        _dev->timer = xTimerCreate
                    (timer_name,
                    pdMS_TO_TICKS(pulse_in_ms),
                    0,
                    (void *)dev,
                    timeout_cb);
#endif
    }

    LOGI(TAG, "%s done\n", current_36xx_type == AW3644 ? "AW3644" : (current_36xx_type == AW36515 ? "AW36515": "LM3644"));

out:
    return ret;
 }

int common_36xx_release(led_dev_t *dev)
{
#if USE_RTOS
    common_led_t* _dev = (common_led_t*)dev->priv;
    if (_dev->timer)
    {
        if (xPortIsInISR())
        {
            xTimerStopFromISR(_dev->timer, pdMS_TO_TICKS(10));
        }
        else
        {
            xTimerStop(_dev->timer, pdMS_TO_TICKS(10));
        }
        xTimerDelete(_dev->timer, pdMS_TO_TICKS(10));
        _dev->timer = NULL;
    }
#endif

    return 0;
}

int common_36xx_enable(led_dev_t *dev, led_channel_t chn)
{
    common_led_t* _dev = (common_led_t*)dev->priv;
    i2c_device_number_t i2c_num = _dev->i2c_num;
    uint8_t i2c_addr = _dev->i2c_addr;
    uint8_t data; 

    common_36xx_get_lock(dev);
    s_chn = chn;

    i2c_init(i2c_num, i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);

    data = common_36xx_read_reg(i2c_num, i2c_addr, ENABLE_REG);

    // switch(led_num) {
    //     case LED_0:
            // data |= 0x1;
    //         break;
    //     case LED_1:
    //         data |= 0x2;
    //         break;
    //     default:
    //         break;
    // }

    if (chn == CHANNEL_0) {
        data |= 0x1;
    } else if (chn == CHANNEL_1) {
        data |= 0x2;
    }

    int ret = common_36xx_write_reg(i2c_num, i2c_addr, ENABLE_REG, data);

    common_36xx_release_lock(dev);

    dev->is_enable[chn] = (ret == 0 ? 1 : 0);

    return ret;
}

int common_36xx_disable(led_dev_t *dev, led_channel_t chn)
{
    common_led_t* _dev = (common_led_t*)dev->priv;
    i2c_device_number_t i2c_num = _dev->i2c_num;
    uint8_t i2c_addr = _dev->i2c_addr;
    uint8_t data; 

    common_36xx_get_lock(dev);

    i2c_init(i2c_num, i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);

    data = common_36xx_read_reg(i2c_num, i2c_addr, ENABLE_REG);

    // switch(led_num) {
    //     case LED_0:
            // data &= ~0x1;
    //         break;
    //     case LED_1:
    //         data &= ~0x2;
    //         break;
    //     default:
    //         break;
    // }

    if (chn == CHANNEL_0) {
        data &= ~0x1;
    } else if (chn == CHANNEL_1) {
        data &= ~0x2;
    }

    int ret = common_36xx_write_reg(i2c_num, i2c_addr, ENABLE_REG, data);

    common_36xx_release_lock(dev);

    if (ret == 0)
        dev->is_enable[chn] = 0;

    return ret;
}

int common_36xx_suspend(led_dev_t *dev, led_channel_t chn)
{
#if (defined USE_SENSOR_OV02B_STROBE)
    int ret = common_36xx_disable(dev,chn);
    dev->is_suspend[chn] = (ret == 0) ? 1 : 0;
#else
    dev->is_suspend[chn] = 1;
#endif

    return 0;
}

int common_36xx_resume(led_dev_t *dev, led_channel_t chn)
{
#if (defined USE_SENSOR_OV02B_STROBE)
    int ret = common_36xx_enable(dev,chn);
    dev->is_suspend[chn] = (ret == 0) ? 0 : 1;
#else
    dev->is_suspend[chn] = 0;
#endif

    return 0;
}

int common_36xx_get_current_range(led_dev_t *dev, led_channel_t chn, float *min, float *max, float *step)
{
    (void)chn;
    common_36xx_get_lock(dev);
    *min = supported_36xx_products[current_36xx_type].current_min_ma;
    *step = supported_36xx_products[current_36xx_type].current_step_ma;
    *max = supported_36xx_products[current_36xx_type].current_min_ma + ((*step)*MAX_BRIGHTNESS);
    common_36xx_release_lock(dev);

    return 0;
}

int common_36xx_set_current_level(led_dev_t *dev, led_current_level_t level)
{
    common_36xx_get_lock(dev);

    common_led_t *_dev = (common_led_t *)dev->priv;
    i2c_device_number_t i2c_num = _dev->i2c_num;
    uint8_t led_index = _dev->led_index;
    i2c_init(i2c_num, _dev->i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);

    led_status[led_index].brightness = ((uint8_t)level) & MAX_BRIGHTNESS;

    uint8_t flag1, flag2;

    flag1 = common_36xx_read_reg(i2c_num, _dev->i2c_addr, FLAGS1_REG);
    flag2 = common_36xx_read_reg(i2c_num, _dev->i2c_addr, FLAGS2_REG);

    if (flag1 || flag2)
    {
        LOGW(__func__, "Fault detected: flag1 [0x%x], flag2 [0x%x]", flag1, flag2);
    }
    uint8_t data = common_36xx_read_reg(i2c_num, _dev->i2c_addr, ENABLE_REG);
    data |= 0x24;
   // LOGI("","ENABLE_REG %x", data);
    int ret = common_36xx_write_reg(i2c_num, _dev->i2c_addr, ENABLE_REG, data);

    if (ret == 0)
    {
        common_36xx_write_reg(i2c_num,
            _dev->i2c_addr,
            led_status[led_index].brightness_reg_addr, 
           led_status[led_index].brightness);
    }

    common_36xx_release_lock(dev);

    return ret;
}

int common_36xx_set_current(led_dev_t *dev, led_channel_t chn, float current_ma)
{
    int ret = -1;

    //
    float current_min_ma = supported_36xx_products[current_36xx_type].current_min_ma;
    float current_step_ma = supported_36xx_products[current_36xx_type].current_step_ma;
    int level = (int)(roundf((current_ma - current_min_ma) / current_step_ma));

    if (level < 0 || level > MAX_BRIGHTNESS)
    {
        LOGE(TAG, "invalid current:%f\n", current_ma);
        return ret;
    }

    LOGD(TAG, "current %f mA, level val:%d\n", current_ma, level);

    ret = common_36xx_set_current_level(dev, level);

    return ret;
}

int common_36xx_set_timeout_ms(led_dev_t *dev, led_channel_t chn, int timeout_ms)
{
    dev->timeout_ms[chn] = timeout_ms;
    enum common_36xx_time_out_val timeout_level = timeout_ms2level(timeout_ms);

    return common_36xx_set_flash_timeout(dev, timeout_level);
}

int common_36xx_trigger(led_dev_t *dev, led_channel_t chn)
{
    configASSERT(dev->priv != NULL);
    common_led_t* _dev = (common_led_t*)dev->priv;
    uint8_t pin = _dev->power_pin;
    const led_trig_param_t *trig = dev->trig_param;

    if (_dev->use_strobe)
    {


        return 0;
    }

    if (!dev->is_suspend[chn])
    {
        if (dev->is_enable[chn])
        {
            gpio_set_pin(pin, GPIO_PV_LOW);
            //
            int pulse_in_ms = 2;
            if (trig != NULL && trig->pulse_in_us >= 2000 && trig->pulse_in_us <= 10000)
            {
                pulse_in_ms = (trig->pulse_in_us / 1000);
            }
#if USE_RTOS
            TimerHandle_t timer = _dev->timer;
            TickType_t period = xTimerGetPeriod(timer);
            TickType_t expect_period = pulse_in_ms / portTICK_PERIOD_MS;
            if (xPortIsInISR())
            {
                if (period != expect_period)
                {
                    xTimerChangePeriodFromISR(timer, expect_period, NULL);
                }
                xTimerStartFromISR(timer, NULL);
            }
            else
            {
                if (period != expect_period)
                {
                    xTimerChangePeriod(timer, expect_period, 0);
                }
                xTimerStart(timer, 0);
            }
#else
            aiva_busy_delay_ms(pulse_in_ms);
            gpio_set_pin(pin, GPIO_PV_HIGH);
#endif
        }
    }

    return 0;
}


#ifdef __cplusplus
}
#endif

static led_ops_t common_36xx_led_ops = {
    .init = common_36xx_init,
    .release = common_36xx_release,
    .enable = common_36xx_enable,
    .disable = common_36xx_disable,
    .suspend = common_36xx_suspend,
    .resume = common_36xx_resume,
    .get_current_range = common_36xx_get_current_range,
    .set_current = common_36xx_set_current,
    .set_timeout = common_36xx_set_timeout_ms,
    .trigger = common_36xx_trigger,
};

static common_led_t led0 = {
    .i2c_num = I2C_DEVICE_1,
    .i2c_addr = LM3644_SLAVE_ADDR,
    .i2c_addr_width = 7,
    .i2c_clk = 50 * 1000,
    .power_pin = GPIO_PIN3,
    .led_index = 0,
// TODO: decoupling from sensor???
#if (defined USE_SENSOR_OV02B_STROBE)
    .use_strobe = 1, // use strobe or gpio
#else
    .use_strobe = 0,
#endif

#if USE_RTOS
    .timer = NULL,
    .mutex = PTHREAD_MUTEX_INITIALIZER,
#endif
};

static led_trig_param_t trig_param = {
    .pulse_in_us = 2000,
};

static led_dev_t lm3644 = {
    .name = "lm3644",
    .ops = &common_36xx_led_ops,
    .trig_param = &trig_param,
    .is_initialized = 0,
    .is_enable = 0,
    .is_suspend = 0,
    // .timeout_ms = 30,
    .timeout_ms = 40, // this vlaue should be the same with {TIMING_REG, 0x13}
    .priv = &led0,
};

//NOTE: use apis in led_driver.h
/* Export Device:
 * 	    [uclass ID], [device name], [flags], [driver], [private pointer]
 */
UDEVICE_EXPORT(UCLASS_LED,  lm3644,  0,  &lm3644,  NULL);
