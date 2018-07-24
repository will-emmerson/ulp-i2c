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

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


const gpio_num_t gpio_led = GPIO_NUM_2;
const gpio_num_t gpio_scl = GPIO_NUM_32;
const gpio_num_t gpio_sda = GPIO_NUM_33;


#define RTC_1S_SLEEP 154750


static void init_ulp_program()
{
    rtc_gpio_init(gpio_led);
    rtc_gpio_set_direction(gpio_led, RTC_GPIO_MODE_OUTPUT_ONLY);

    rtc_gpio_init(gpio_scl);
    rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(gpio_sda);
    rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

    ESP_ERROR_CHECK(ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)));

    /* Set ULP wake up period to T = 5 seconds */
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 150000);

}


#define t1 ((uint16_t)ulp_t1)
#define t2 ((int16_t) ulp_t2)
#define t3 ((int16_t) ulp_t3)
#define p1 ((uint16_t)ulp_p1)
#define p2 ((int16_t) ulp_p2)
#define p3 ((int16_t) ulp_p3)
#define p4 ((int16_t) ulp_p4)
#define p5 ((int16_t) ulp_p5)
#define p6 ((int16_t) ulp_p6)
#define p7 ((int16_t) ulp_p7)
#define p8 ((int16_t) ulp_p8)
#define p9 ((int16_t) ulp_p9)


static void print_status()
{
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

    /* Start the program */
    esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    esp_deep_sleep_start();

}
