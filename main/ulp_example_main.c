/* ULP I2C bit bang BMP-180 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <math.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "ulp_main.h"


/*
# OSRS_P = 1 # 16 Bit ultra low power
# OSRS_P = 2 # 17 Bit low power
# OSRS_P = 3 # 18 Bit standard resolution
# OSRS_P = 4 # 19 Bit high resolution
# OSRS_P = 5 # 20 Bit ultra high resolution

# OSRS_T = 0 # skipped
# OSRS_T = 1 # 16 Bit
# OSRS_T = 2 # 17 Bit
# OSRS_T = 3 # 18 Bit
# OSRS_T = 4 # 19 Bit
# OSRS_T = 5 # 20 Bit

# FILTER = 0 #
# FILTER = 1 #
# FILTER = 2 #
# FILTER = 3 #
# FILTER = 4 #
# FILTER = 5 #
# FILTER = 6 #
# FILTER = 7 #

# standby settings
# T_SB = 0 # 000 0,5ms
# T_SB = 1 # 001 62.5 ms
# T_SB = 2 # 010 125 ms
# T_SB = 3 # 011 250ms
# T_SB = 4 # 100 500ms
# T_SB = 5 # 101 1000ms
# T_SB = 6 # 110 2000ms
# T_SB = 7 # 111 4000ms

# power mode
# POWER_MODE=0 # sleep mode
# POWER_MODE=1 # forced mode
# POWER_MODE=2 # forced mode
# POWER_MODE=3 # normal mode

 */

#define OSRS_P 5
#define OSRS_T 2
#define T_SB 7
#define POWER_MODE 1
#define FILTER 2

#define CONFIG  ((T_SB <<5) + (FILTER <<2))
#define CTRL_MEAS ((OSRS_T <<5) + (OSRS_P <<2) + POWER_MODE)




extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


//const gpio_num_t gpio_led = GPIO_NUM_2;
const gpio_num_t gpio_scl = GPIO_NUM_32;
const gpio_num_t gpio_sda = GPIO_NUM_33;


#define RTC_1S_SLEEP 154750


static void init_ulp_program()
{

    printf("ulp_config = %x\n", CONFIG);
    printf("ulp_ctrl = %x\n", CTRL_MEAS);
    ulp_reg_config = CONFIG;
    ulp_reg_ctrl= CTRL_MEAS;
    rtc_gpio_init(gpio_scl);
    rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(gpio_sda);
    rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

    ESP_ERROR_CHECK(ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)));

    /* Set ULP wake up period to T = 5 seconds */
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 5*RTC_1S_SLEEP);

}


#define t1 ((uint16_t)ulp_t1)
#define t2 ((int16_t) ulp_t2)
#define t3 ((int16_t) ulp_t3)
#define p1 ((int16_t)ulp_p1)
#define p2 ((int16_t) ulp_p2)
#define p3 ((int16_t) ulp_p3)
#define p4 ((int16_t) ulp_p4)
#define p5 ((int16_t) ulp_p5)
#define p6 ((int16_t) ulp_p6)
#define p7 ((int16_t) ulp_p7)
#define p8 ((int16_t) ulp_p8)
#define p9 ((int16_t) ulp_p9)

#define temp_a ((uint8_t) ulp_temp_a)
#define temp_b ((uint8_t) ulp_temp_b)
#define temp_c ((uint8_t) ulp_temp_c)

#define BME280_S32_t int32_t
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t

static BME280_S32_t bme280_t_fine;

static BME280_S32_t bme280_compensate_T(BME280_S32_t adc_T) {
    BME280_S32_t var1, var2, T;
    var1  = ((((adc_T>>3) - ((BME280_S32_t)t1<<1))) * ((BME280_S32_t)t2)) >> 11;
    var2  = (((((adc_T>>4) - ((BME280_S32_t)t1)) * ((adc_T>>4) - ((BME280_S32_t)t1))) >> 12) *
             ((BME280_S32_t)t3)) >> 14;
    bme280_t_fine = var1 + var2;
    T  = (bme280_t_fine * 5 + 128) >> 8;
    return T;
}




static void print_status()
{

    // Must do Temp first since bme280_t_fine is used by the other compensation functions

    BME280_S32_t temp ;
    uint32_t adc_T = (uint32_t)(((temp_a << 16) | (temp_b << 8) | temp_c) >> 4);
    if (adc_T == 0x80000 || adc_T == 0xfffff) {
        temp = 0;
    } else {
        temp = bme280_compensate_T(adc_T);
    }

    printf("ADC_T: %i\n", adc_T);
    printf("Temp: %i\n", temp);

    printf("%d ", t1);
    printf("%d ", t2);
    printf("%d \n", t3);
    printf("%d ", p1);
    printf("%d ", p2);
    printf("%d ", p3);
    printf("%d ", p4);
    printf("%d ", p5);
    printf("%d ", p6);
    printf("%d ", p7);
    printf("%d ", p8);
    printf("%d ", p9);
    printf("\n");
    printf("%d ", temp_a);
    printf("%d ", temp_b);
    printf("%d \n", temp_c);

}

void app_main()
{

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
    } else {
    	printf("ULP wakeup, printing status\n");
        print_status();
    }

    printf("Entering deep sleep\n\n");

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );

    esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    vTaskDelay(10);

    esp_deep_sleep_start();

}
