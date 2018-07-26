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
#include "esp_wifi.h"
#include "soc/rtc.h"
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
 BMP280 Config options.


OSRS_P = 1 # 16 Bit ultra low power
OSRS_P = 2 # 17 Bit low power
OSRS_P = 3 # 18 Bit standard resolution
OSRS_P = 4 # 19 Bit high resolution
OSRS_P = 5 # 20 Bit ultra high resolution

OSRS_T = 0 # skipped
OSRS_T = 1 # 16 Bit
OSRS_T = 2 # 17 Bit
OSRS_T = 3 # 18 Bit
OSRS_T = 4 # 19 Bit
OSRS_T = 5 # 20 Bit

FILTER = 0 #
FILTER = 1 #
FILTER = 2 #
FILTER = 3 #
FILTER = 4 #
FILTER = 5 #
FILTER = 6 #
FILTER = 7 #

standby settings (not used in forced mode)
T_SB = 0 # 000 0,5ms
T_SB = 1 # 001 62.5 ms
T_SB = 2 # 010 125 ms
T_SB = 3 # 011 250ms
T_SB = 4 # 100 500ms
T_SB = 5 # 101 1000ms
T_SB = 6 # 110 2000ms
T_SB = 7 # 111 4000ms

power mode
POWER_MODE=0 # sleep mode
POWER_MODE=1 # forced mode
POWER_MODE=2 # forced mode
POWER_MODE=3 # normal mode

 */

#define OSRS_P 5
#define OSRS_T 2
#define FILTER 2

// Don't change these next two - the ULP code relies on the sensor operating in FORCED mode...
#define T_SB 7
#define POWER_MODE 1

#define CONFIG  ((T_SB <<5) + (FILTER <<2))
#define CTRL_MEAS ((OSRS_T <<5) + (OSRS_P <<2) + POWER_MODE)

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

//LOLIN32 Lite has built in LED on GPIO 22
#define LED_GPIO GPIO_NUM_22

// You must use RTC_IOs for SCL/SDA - not all GPIO are supported by the RTC/ULP...
const gpio_num_t gpio_scl = GPIO_NUM_32;
const gpio_num_t gpio_sda = GPIO_NUM_33;

/*
  Configure how often the ULP should read the sensor. Note that the ULP wakes up 10 times
  before it makes a reading, so if you want a reading every 10 seconds, put 1 in the SECONDS_PER_ULP_WAKEUP
  or modify the line in main.S that reads jumpr waitNext,10,lt // halt if r0 < 10
  */

//#define SLEEP_CYCLES_PER_S 187500 // cycles per second
#define SLEEP_CYCLES_PER_S 18750 // cycles per second
#define SECONDS_PER_ULP_WAKEUP 1

static void setup()
{
//    printf("ulp_config = %x\n", CONFIG);
//    printf("ulp_ctrl = %x\n", CTRL_MEAS);

    // pass the BMP config and ctrl register values to the ULP
    ulp_reg_config = CONFIG;
    ulp_reg_ctrl= CTRL_MEAS;
    rtc_gpio_init(gpio_scl);
    rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(gpio_sda);
    rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

    ESP_ERROR_CHECK(ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)));

    /* Set ULP wake up period to T = 5 seconds */
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, (SECONDS_PER_ULP_WAKEUP*SLEEP_CYCLES_PER_S));

}

// BMP280 calibration data read by the ULP

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
#define temp_a ((uint8_t) ulp_temp_msb)
#define temp_b ((uint8_t) ulp_temp_lsb)
#define temp_c ((uint8_t) ulp_temp_xlsb)
#define counter ((uint16_t) ulp_counter)

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
    printf("Counter: %d\n", counter);
    ulp_counter = 0;

    //pressure to be implemented

//    printf("%d ", t1);
//    printf("%d ", t2);
//    printf("%d \n", t3);
//    printf("%d ", p1);
//    printf("%d ", p2);
//    printf("%d ", p3);
//    printf("%d ", p4);
//    printf("%d ", p5);
//    printf("%d ", p6);
//    printf("%d ", p7);
//    printf("%d ", p8);
//    printf("%d ", p9);
//    printf("\n");
//    printf("%d ", temp_a);
//    printf("%d ", temp_b);
//    printf("%d \n", temp_c);

}

// For really low power, check whether your board has a pull up resistor on the SPI Flash
// SPI Flash requires a pull-up resistor for CS pin, so it goes into standby mode when ESP is in deep sleep.
//I've put a 100k pull-up resistor and current dropped from 900µA to 75µA with built-in LDO (55µA quiescent current).
// from https://forum.sparkfun.com/viewtopic.php?t=45931#p194837

void app_main()
{
    //Turn on LED while running
    gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LED_GPIO, GPIO_PULLDOWN_ONLY);
    gpio_set_level(LED_GPIO, 1);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup, initializing ULP\n");
        setup();
    } else {
    	printf("ULP wakeup, printing status\n");
        print_status();
    }

    printf("Preparing for deep sleep\n\n");

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    ESP_ERROR_CHECK( ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)) );

    printf("Entering deep sleep\n\n");
    vTaskDelay(20);

    gpio_set_level(LED_GPIO, 0);
    esp_deep_sleep_start();

}
