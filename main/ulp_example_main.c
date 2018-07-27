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
#define SLEEP_CYCLES_PER_S rtc_clk_slow_freq_get_hz() // cycles per second
#define SECONDS_PER_ULP_WAKEUP 2

RTC_DATA_ATTR static unsigned int boot_count = 0;
RTC_DATA_ATTR static unsigned long wake_millis = 0;

static void setup()
{
    printf("ulp_config = 0x%x\n", CONFIG);
    printf("ulp_ctrl = 0x%x\n", CTRL_MEAS);

    // pass the BMP config and ctrl register values to the ULP
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
#define dig_P1 ((uint16_t)ulp_p1)
#define dig_P2 ((int16_t) ulp_p2)
#define dig_P3 ((int16_t) ulp_p3)
#define dig_P4 ((int16_t) ulp_p4)
#define dig_P5 ((int16_t) ulp_p5)
#define dig_P6 ((int16_t) ulp_p6)
#define dig_P7 ((int16_t) ulp_p7)
#define dig_P8 ((int16_t) ulp_p8)
#define dig_P9 ((int16_t) ulp_p9)
#define temp_msb ((uint8_t) ulp_temp_msb)
#define temp_lsb ((uint8_t) ulp_temp_lsb)
#define temp_xlsb ((uint8_t) ulp_temp_xlsb)
#define pres_msb ((uint8_t) ulp_pres_msb)
#define pres_lsb ((uint8_t) ulp_pres_lsb)
#define pres_xlsb ((uint8_t) ulp_pres_xlsb)


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

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
float bme280_compensate_P(BME280_S32_t adc_P) {

    float var1, var2, p;

    var1=bme280_t_fine/2.0-64000.0;
    var2=var1*var1*dig_P6/32768.0;
    var2=var2+var1*dig_P5*2;
    var2=var2/4.0+dig_P4*65536.0;
    var1=(dig_P3*var1*var1/524288.0+dig_P2*var1)/524288.0;
    var1=(1.0+var1/32768.0)*dig_P1;

    p=1048576.0-adc_P;
    p=(p-var2/4096.0)*6250.0/var1;
    var1=dig_P9*p*p/2147483648.0;
    var2=p*dig_P8/32768.0;
    p=p+(var1+var2+dig_P7)/16.0;

    return p;
}


static void print_status()
{

    // Must do Temp first since bme280_t_fine is used by the other compensation functions

    float temp, press;
    uint32_t adc_T = (uint32_t)(((temp_msb << 16) | (temp_lsb << 8) | temp_xlsb) >> 4);
    uint32_t adc_P = (uint32_t)(((pres_msb << 16) | (pres_lsb << 8) | pres_xlsb) >> 4);

    if (adc_T == 0x80000 || adc_T == 0xfffff) {
        temp = 0;
    } else {
        temp = bme280_compensate_T(adc_T);
    }

    if (adc_P ==0x80000 || adc_P == 0xfffff) {
        press = 0;
    } else {
        press = bme280_compensate_P(adc_P);
    }

    boot_count += 1;
    printf("Boot count: %u\n", boot_count);
    printf("Total wake s: %lu\n", wake_millis/1000);
    printf("Temp: %.2f C\n", temp/100.0);
    printf("Pres: %.2f hPa\n", press/100.0);

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
//    printf("temp raw : 0x%x 0x%x 0x%x\n", temp_msb, temp_lsb, temp_xlsb);
//    printf("press raw : 0x%x 0x%x 0x%x\n", pres_msb, pres_lsb, pres_xlsb);

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
        printf("First start - initializing ULP\n");
        setup();
    } else {
        print_status();
    }

    ulp_reg_config = CONFIG;
    ulp_reg_ctrl= CTRL_MEAS;

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    ESP_ERROR_CHECK( ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)) );

    printf("Entering deep sleep\n\n");
    vTaskDelay(20);
    wake_millis +=  esp_timer_get_time()/1000;
    gpio_set_level(LED_GPIO, 0);
    esp_deep_sleep_start();

}
