/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sys/cdefs.h>
#include "esp_err.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "driver/uart.h"

#include "cJSON.h"
#include "cJSON_Utils.h"

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN                    256

static adc_channel_t channel[1] = {ADC_CHANNEL_4};

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

#define THRESH 16
#define POTS_NUM 1
#define SWS_NUM 6
#define UISW_NUM 4
#define LEDS_NUM 6

#define NUM_PINS 3  // Number of GPIOs to read

// Define the GPIO pins to read
const gpio_num_t gpio_pins_pedals_in[SWS_NUM] = {GPIO_NUM_5, GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23};
const gpio_num_t gpio_pins_ui_in[UISW_NUM] = {GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_16, GPIO_NUM_19};
                                            //next bank    prev_bank    next preset     prev preset
const gpio_num_t gpio_pins_leds[LEDS_NUM] = {GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13};

// Function to configure GPIO pins as input with pull-down enabled
void configure_pins() {
    for (int i = 0; i < SWS_NUM; i++) {
        gpio_reset_pin(gpio_pins_pedals_in[i]);
        gpio_pulldown_dis(gpio_pins_pedals_in[i]);   // Ensure pull-up is disabled
        gpio_pullup_en(gpio_pins_pedals_in[i]);  // Enable internal pull-down resistor
        gpio_set_direction(gpio_pins_pedals_in[i], GPIO_MODE_INPUT);
    }
    for (int i = 0; i < UISW_NUM; i++) {
        gpio_reset_pin(gpio_pins_ui_in[i]);
        gpio_pulldown_dis(gpio_pins_ui_in[i]);   // Ensure pull-up is disabled
        gpio_pullup_en(gpio_pins_ui_in[i]);  // Enable internal pull-down resistor
        gpio_set_direction(gpio_pins_ui_in[i], GPIO_MODE_INPUT);
    }
    for (int i = 0; i < LEDS_NUM; i++) {
        gpio_reset_pin(gpio_pins_leds[i]);
        gpio_set_direction(gpio_pins_leds[i], GPIO_MODE_OUTPUT);
        gpio_pullup_dis(gpio_pins_leds[i]);  // Enable internal pull-down resistor
        gpio_pulldown_en(gpio_pins_leds[i]);   // Ensure pull-up is disabled
    }
}

typedef enum UI_ACTION {
    UI_ACTION_NEXT_BANK = 0,
    UI_ACTION_PREV_BANK,
    UI_ACTION_NEXT_PRES,
    UI_ACTION_PREV_PRES,

    UI_ACTION_NONE
} ui_action_enum_t;

// Function to read the state of GPIO pins
void read_gpio(uint8_t *pins_values, ui_action_enum_t *action) {
    for (int i = 0; i < SWS_NUM; i++) {
        pins_values[i] = gpio_get_level(gpio_pins_pedals_in[i]);
        //printf("GPIO %d: %d\n", gpio_pins[i], level);
    }

    for (ui_action_enum_t act = 0; act < UI_ACTION_NONE; act++) {
        if (gpio_get_level(gpio_pins_ui_in[act]) == 0) {
            *action = act;
            return;
        }
    }
}

void ui_action_to_str(ui_action_enum_t act, [[maybe_unused]]char **out) {
    *out = "none";
    switch (act)
    {
        case UI_ACTION_NEXT_BANK:
            *out = "nxbk";
            break;
        case UI_ACTION_PREV_BANK:
            *out = "pxbk";
            break;
        case UI_ACTION_NEXT_PRES:
            *out = "nxps";
            break;
        case UI_ACTION_PREV_PRES:
            *out = "pxps";
            break;
        case UI_ACTION_NONE:
            *out = "none";
            break;
    }
    //ESP_LOGI(__func__, "act: %s", out);
}


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void apply_led_states(uint8_t *led_states) {
    for (int led = 0; led < LEDS_NUM; led++) {
        gpio_set_level(gpio_pins_leds[led], led_states[led]);
        ESP_LOGI(__func__, "led on gpio [%d] set to [%d]", gpio_pins_leds[led], led_states[led]);
    }
}

#define RX_BUF_SIZE 2048
#define TX_BUF_SIZE 2048
#define QUEUE_SIZE 2048

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(0, RX_BUF_SIZE, TX_BUF_SIZE, QUEUE_SIZE, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(0, GPIO_NUM_1, GPIO_NUM_3, -1, -1));
    uint8_t *data_rx = (uint8_t*) malloc(RX_BUF_SIZE);
    uint8_t *data_tx = (uint8_t*) malloc(TX_BUF_SIZE);
    uint8_t *data_queue = (uint8_t*) malloc(QUEUE_SIZE);

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    configure_pins();
    uint32_t last_sent = 0;

    uint32_t pots_vals[POTS_NUM] = {0};
    uint8_t sw_vals[SWS_NUM] = {0, 0, 0, 0, 0};
    uint8_t uisw_vals[UISW_NUM] = {0, 0, 0, 0};
    uint8_t leds_states[LEDS_NUM] = {0, 0, 0, 0, 0, 0};
    char pedal_names[64][6];

    char *current_bank_buf = calloc(64, 1);
    char *current_preset = calloc(64, 1);

    char *curr_action = calloc(5, 1);
    curr_action = "none";
    ui_action_enum_t curr_action_enum = UI_ACTION_NONE;
    ui_action_enum_t last_action_enum = UI_ACTION_NONE;

    char *read_from_serial_buf = (char*) calloc(1024, 1);
    if (!read_from_serial_buf) {
        ESP_LOGE("CALLOC FAIL", "read_from_serial_buf");
    }
    
    while (1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1) {
            bool needs_update = true;
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        //ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                        //data = data & (0xffffffff << 3);
                        if ((data > last_sent + THRESH) || (data < last_sent - THRESH)) {
                            last_sent = data;
                            pots_vals[0] = data;
                            needs_update = true;
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }

                read_gpio(sw_vals, &curr_action_enum);
                if (last_action_enum == UI_ACTION_NONE) {
                    ui_action_to_str(curr_action_enum, &curr_action);
                } else {
                    curr_action = "none";
                }
                last_action_enum = curr_action_enum;
                //ESP_LOGI("ACT", "%s", curr_action);

                if (needs_update) {

                    cJSON *pedal_switches = cJSON_CreateArray();
                    cJSON *pots_values = cJSON_CreateArray();
                    cJSON *ui_action = cJSON_CreateString(curr_action);

                    for (int i = 0; i < POTS_NUM; i++) {
                        cJSON *tmp_item = cJSON_CreateNumber(pots_vals[i]);
                        cJSON_AddItemToArray(pots_values, tmp_item);
                    }
                    for (int i = 0; i < SWS_NUM; i++) {
                        cJSON *tmp_item = cJSON_CreateNumber(sw_vals[i]);
                        cJSON_AddItemToArray(pedal_switches, tmp_item);
                    }
                    cJSON *dataOut = cJSON_CreateObject();
                    cJSON_AddItemToObject(dataOut, "pedals", pedal_switches);
                    cJSON_AddItemToObject(dataOut, "pots", pots_values);
                    cJSON_AddItemToObject(dataOut, "ui_action", ui_action);
                    char *dataOut_buf = cJSON_PrintUnformatted(dataOut);
                    uart_write_bytes(0, dataOut_buf, strlen(dataOut_buf));
                    uart_write_bytes(0, "\n", 1);

//                    printf("%s\n", cJSON_PrintUnformatted(dataOut));
                    cJSON_Delete(dataOut);
                    free(dataOut_buf);
                }
                curr_action_enum = UI_ACTION_NONE;

                cJSON *recv_json = NULL;

                int len = uart_read_bytes(0, data_rx, RX_BUF_SIZE -1, pdMS_TO_TICKS(100));
                if (len) {
                    data_rx[len] = '\0';
                    ESP_LOGI(TAG, "Recv str: %s", (char *) data_rx);
                    recv_json = cJSON_Parse((char*) data_rx);
                }

                if (recv_json != NULL) {

                    ESP_LOGI("JSON READ", "success");
                    
                    cJSON *pedal_arr_j = cJSON_GetObjectItemCaseSensitive(recv_json, "pedals_onoff");
                    for (int led = 0; led < cJSON_GetArraySize(pedal_arr_j);led++){
                        cJSON *eff_state_tuple = cJSON_GetArrayItem(pedal_arr_j, led);
                        cJSON *pedal_state_j = cJSON_GetObjectItemCaseSensitive(eff_state_tuple, "val");
                        leds_states[led] = cJSON_GetNumberValue(pedal_state_j);
                        //ESP_LOGI("cJSON delete", "tuple and pedal state");
                        //cJSON_Delete(eff_state_tuple);
                        //cJSON_Delete(pedal_state_j);
                    }
                    //ESP_LOGI("cJSON delete", "pedal array");
                    cJSON_Delete(pedal_arr_j);
                    apply_led_states(leds_states);
                } else {
                    //ESP_LOGW("JSON READ", "FAIL");
                }
                
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
