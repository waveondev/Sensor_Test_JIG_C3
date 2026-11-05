#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

/* GPIO */
#define HX711_SCK       GPIO_NUM_18
#define HX711_DOUT      GPIO_NUM_19

#define OFFSET_BUTTON   GPIO_NUM_2
#define LED_GPIO        GPIO_NUM_5
#define OUTPUT_GPIO     GPIO_NUM_1

/* ADC */
#define MOTOR_ADC_CH    ADC_CHANNEL_0
#define LEFT_IR_ADC_CH  ADC_CHANNEL_3
#define RIGHT_IR_ADC_CH ADC_CHANNEL_4

/* Timing */
#define SAMPLE_PERIOD_MS    50
#define ADC_AVG_COUNT       10

/* Loadcell thresholds */
#define LOADCELL_PRINT_THRESHOLD 5000
#define LOADCELL_PASS_THRESHOLD  50000
#define LOADCELL_FAIL_THRESHOLD -50000

static const char *TAG = "JIG";

static adc_oneshot_unit_handle_t adc_handle;

/* OFFSET flag */
static bool offset_flag = false;
static bool offset_ready = false;

/* SWITCH previous state */
static int switch_previous = 1;

/* Loadcell */
static int32_t loadcell_offset = 0;
static int32_t loadcell_value = 0;
static int32_t last_printed_value = 0;

static bool loadcell_valid = false;
static bool last_value_valid = false;

/* 0=READY, 1=PASS, 2=FAIL */
static int loadcell_result = 0;
static int last_printed_result = -1;

/* ADC accumulation */
static int64_t motor_sum = 0;
static int64_t left_ir_sum = 0;
static int64_t right_ir_sum = 0;

static uint32_t motor_count = 0;
static uint32_t left_ir_count = 0;
static uint32_t right_ir_count = 0;

static uint32_t sample_count = 0;

static int motor_average = 0;
static int left_ir_average = 0;
static int right_ir_average = 0;


/* GPIO 초기화 */
static void jig_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << OFFSET_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));

    cfg.pin_bit_mask = (1ULL << LED_GPIO) |
                       (1ULL << OUTPUT_GPIO);

    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&cfg));

    /* GPIO5 LED OFF */
    ESP_ERROR_CHECK(gpio_set_level(LED_GPIO, 0));

    /* GPIO1 HIGH */
    ESP_ERROR_CHECK(gpio_set_level(OUTPUT_GPIO, 1));

    switch_previous = gpio_get_level(OFFSET_BUTTON);
}


/* HX711 초기화 */
static void hx711_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << HX711_SCK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));

    cfg.pin_bit_mask = (1ULL << HX711_DOUT);
    cfg.mode = GPIO_MODE_INPUT;

    ESP_ERROR_CHECK(gpio_config(&cfg));

    ESP_ERROR_CHECK(gpio_set_level(HX711_SCK, 0));
}


/* HX711 RAW 데이터 읽기 */
static bool hx711_read(int32_t *value)
{
    uint32_t data = 0;

    if (gpio_get_level(HX711_DOUT) != 0) {
        return false;
    }

    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK, 1);
        esp_rom_delay_us(1);

        data = (data << 1) |
               (uint32_t)gpio_get_level(HX711_DOUT);

        gpio_set_level(HX711_SCK, 0);
        esp_rom_delay_us(1);
    }

    /* Gain 128 */
    gpio_set_level(HX711_SCK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK, 0);
    esp_rom_delay_us(1);

    /* Sign extension */
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    *value = (int32_t)data;

    return true;
}


/* ADC 초기화 */
static void jig_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&init_config, &adc_handle)
    );

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle, MOTOR_ADC_CH, &channel_config)
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle, LEFT_IR_ADC_CH, &channel_config)
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle, RIGHT_IR_ADC_CH, &channel_config)
    );
}


/* ADC 측정 */
static void jig_adc_read_task(void)
{
    int value;

    if (adc_oneshot_read(adc_handle, MOTOR_ADC_CH, &value) == ESP_OK) {
        motor_sum += value;
        motor_count++;
    }

    if (adc_oneshot_read(adc_handle, LEFT_IR_ADC_CH, &value) == ESP_OK) {
        left_ir_sum += value;
        left_ir_count++;
    }

    if (adc_oneshot_read(adc_handle, RIGHT_IR_ADC_CH, &value) == ESP_OK) {
        right_ir_sum += value;
        right_ir_count++;
    }

    sample_count++;

    if (sample_count >= ADC_AVG_COUNT) {

        if (motor_count > 0) {
            motor_average = (int)(motor_sum / motor_count);
        }

        if (left_ir_count > 0) {
            left_ir_average = (int)(left_ir_sum / left_ir_count);
        }

        if (right_ir_count > 0) {
            right_ir_average = (int)(right_ir_sum / right_ir_count);
        }

        motor_sum = 0;
        left_ir_sum = 0;
        right_ir_sum = 0;

        motor_count = 0;
        left_ir_count = 0;
        right_ir_count = 0;

        sample_count = 0;
    }
}


/* SWITCH HIGH -> LOW 감지 */
static void switch_task(void)
{
    int switch_now = gpio_get_level(OFFSET_BUTTON);

    if ((switch_previous == 1) && (switch_now == 0)) {

        vTaskDelay(pdMS_TO_TICKS(30));

        /* 디바운스 후 LOW 확인 */
        if (gpio_get_level(OFFSET_BUTTON) == 0) {
            offset_flag = true;
        }
    }

    switch_previous = gpio_get_level(OFFSET_BUTTON);
}


/* OFFSET 저장 */
static void loadcell_offset_task(void)
{
    if (!offset_flag) {
        return;
    }

    int32_t raw_value;

    /* 정상 RAW 데이터 수신 시 OFFSET 저장 */
    if (hx711_read(&raw_value)) {

        loadcell_offset = raw_value;
        loadcell_value = raw_value;

        offset_ready = true;

        /* OFFSET 저장 완료 후 flag 해제 */
        offset_flag = false;

        last_value_valid = false;
        last_printed_result = -1;

        loadcell_valid = true;

        ESP_LOGI(TAG, "LOADCELL OFFSET : %ld, %s",
                 (long)loadcell_offset,
                 gpio_get_level(OFFSET_BUTTON) == 0 ? "true" : "false");
    }
}


/* Loadcell 측정 및 판정 */
static void loadcell_read_task(void)
{
    int32_t raw_value;
    int64_t delta;

    if (!offset_ready || offset_flag) {
        return;
    }

    if (!hx711_read(&raw_value)) {
        return;
    }

    loadcell_value = raw_value;
    loadcell_valid = true;

    delta = (int64_t)loadcell_value -
            (int64_t)loadcell_offset;

    if (delta >= LOADCELL_PASS_THRESHOLD) {
        loadcell_result = 1;
    } else if (delta <= LOADCELL_FAIL_THRESHOLD) {
        loadcell_result = 2;
    } else {
        loadcell_result = 0;
    }
}


/* 통합 로그 출력 */
static void jig_log_task(void)
{
    bool switch_pressed =
        (gpio_get_level(OFFSET_BUTTON) == 0);

    ESP_LOGI(TAG, "======================================");

    ESP_LOGI(TAG, "LOADCELL OFFSET : %ld, %s",
             (long)loadcell_offset,
             switch_pressed ? "true" : "false");

    if (offset_ready && loadcell_valid) {

        int64_t value_change = 0;

        if (last_value_valid) {
            value_change = (int64_t)loadcell_value -
                           (int64_t)last_printed_value;

            if (value_change < 0) {
                value_change = -value_change;
            }
        }

        if (!last_value_valid ||
            value_change > LOADCELL_PRINT_THRESHOLD ||
            loadcell_result != last_printed_result) {

            const char *result_text = "READY";

            if (loadcell_result == 1) {
                result_text = "PASS";
            } else if (loadcell_result == 2) {
                result_text = "FAIL";
            }

            ESP_LOGI(TAG, "LOADCELL VALUE : %ld, %s",
                     (long)loadcell_value,
                     result_text);

            last_printed_value = loadcell_value;
            last_value_valid = true;
            last_printed_result = loadcell_result;
        }
    } else {
        ESP_LOGI(TAG, "LOADCELL VALUE : --, READY");
    }

    ESP_LOGI(TAG,
             "MOTOR ADC : %d, LEFT IR : %d, RIGHT IR : %d",
             motor_average,
             left_ir_average,ㄸㄲㄲ
             right_ir_average);

    ESP_LOGI(TAG, "======================================");
}


void app_main(void)
{
    hx711_init();
    jig_gpio_init();
    jig_adc_init();

    ESP_LOGI(TAG, "PROCESS START");

    int log_counter = 0;

    while (1) {

        /* GPIO7 SWITCH edge detection */
        switch_task();

        /* OFFSET 저장 */
        loadcell_offset_task();

        /* ADC 50ms 측정 */
        jig_adc_read_task();

        /* Loadcell 측정 */
        loadcell_read_task();

        /* 10회 측정 후 통합 로그 */
        log_counter++;

        if (log_counter >= ADC_AVG_COUNT) {
            jig_log_task();
            log_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
