#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_system.h"
#include "clock.h"
#include "ld2420_comm.h"

/* Shared variables */

/* Motion detection condition is defined in the software as Distance < Threshold. */
//volatile int32_t motionTimeout = 20;
volatile int32_t motionThreshold = 200;
volatile int motionDetected = 0;
volatile uint32_t ld2420_Distance = 0;


/* Local constants and variables */
static const int RX_BUF_SIZE = 512;
static const int TX_BUF_SIZE = 256;

#define TXD_PIN (GPIO_NUM_24)
#define RXD_PIN (GPIO_NUM_23)

#define LD2420_POWER_PIN (GPIO_NUM_12)
#define POWER_PIN_ON_LEVEL (0)

#define MAX_TX_BUF_WORDS 32

static const char *RX_TASK_TAG = "RX_TASK";

/* Timer and time watch data */

static gptimer_handle_t timer = 0;
static const uint32_t timer_resolution = 10 * 1000;

enum _timestamps {
    TS_COMM,    /* Communication timeout, no data received */
    TS_PARAM,   /* Parameter timeout, parameter not received */
    TS_STALL,   /* Value has not changed for too long */
    TS_STALL11, /* Value is fluctuating around 11 for too long */
//    TS_DETECT,  /* Motion detector: time to keep the motion detection flag */
    TS_count    /* Timestamps count */
};

uint64_t timestamps[TS_count] = {0};

uint64_t timeouts[TS_count] = {
    [TS_COMM]    = timer_resolution * 2,
    [TS_PARAM]   = timer_resolution * 1,
    [TS_STALL]   = timer_resolution * 20,
    [TS_STALL11] = timer_resolution * 30,
//    [TS_DETECT]  = timer_resolution * 100,
};

/* UART Comm data */
static uint32_t range_value = 0;
static uint32_t average_value = 0;
static unsigned measurement_count = 0;

static const int config_read_resp = 0x108;
static const int config_write_resp = 0x107;

struct {
    uint32_t value;
    uint32_t need_update:1;
} abd_parameters[5] = {
    [0] = {   /* min gate */
        .value = 0,
    },
    [1] = {  /* max gate */
        .value = 12,
    },
    [2] = {   /* activeFrameNum (?) */
        .value = 1,
    },
    [3] = {  /* inActiveFrameNum (?) */
        .value = 10,
    },
    [4] = { /* Timeout, s */
        .value = 1,
        .need_update = 1,
    },
};

int abd_param_index = -1;
uint8_t param_write_status = 0;
uint64_t param_time = 0; /* last time parameter access command was sent, read or write */
static const uint64_t param_update_time = timer_resolution * 10; /* 10s */

static void init_gpio(void)
{
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LD2420_POWER_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    gpio_config(&io_conf);
    gpio_set_level(LD2420_POWER_PIN, POWER_PIN_ON_LEVEL);
}

#if 0
esp_err_t init_timer(void)
{
    const gptimer_config_t config = {
        .resolution_hz = timer_resolution,
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
    };

//    if (ESP_OK != gptimer_new_timer(&config, &timer)) {
//        timer = 0;
//        return;
//    }
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&config, &timer), RX_TASK_TAG, "timer");
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
    return ESP_OK;
}

uint64_t clock(void)
{
    uint64_t result = 0;

    if (ESP_OK != gptimer_get_raw_count(timer, &result))
        result = -1;

    return result;
}
#else
#define clock(...) gpclock(timer)
#endif



void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int uart_get_char(int cnt)
{
    int result = 0;

    if (cnt > sizeof(result))
        cnt = sizeof(result);

    const int rxBytes = uart_read_bytes(UART_NUM_1, &result, cnt, 100 / portTICK_PERIOD_MS);

    if (rxBytes <= 0)
        return -1;

    timestamps[TS_COMM] = clock();
    return result;
}


static void normal_range(void)
{
    int ch = uart_get_char(1);

    int mdetect = 0;

    while (ch >= 0) {
        if (ch < '0' || ch > '9') {
    /* Finish reading */
	    average_value += range_value;
	    measurement_count += 1;
//            ESP_LOGI(RX_TASK_TAG, "Value: %lu, %lu", range_value, average_value);

            uint64_t clock_now = clock();

/* Monitor sensor readings to make sure they are not stuck at the same value */
            if (average_value == 0 || range_value != ld2420_Distance) {
                timestamps[TS_STALL] = clock_now;
            }

	    if (8 == measurement_count) {
	        average_value /= 8;

	        ld2420_Distance = average_value;

	        if (average_value == 0 || average_value > 14) {
	            timestamps[TS_STALL11] = clock_now;
	        }

	        mdetect = (average_value > 11 && average_value < motionThreshold);

	        if (mdetect != motionDetected)
	            ESP_LOGI(RX_TASK_TAG, "Motion detected: %s", mdetect ? "On" : "Off");

	        motionDetected = mdetect;


//	            timestamps[TS_DETECT] = clock_now;

//                ESP_LOGI(RX_TASK_TAG, "Motion timer: %lu", (uint32_t)(clock() - timestamps[TS_DETECT]));
//                ESP_LOGI(RX_TASK_TAG, "Clock: %lu", (uint32_t)clock());
//                ESP_LOGI(RX_TASK_TAG, "Avg: %lu Clock: %lu", average_value, (uint32_t)clock());
	        average_value = 0;
	        measurement_count = 0;
	    }
	    range_value = 0;
	    break;
        } else {
    /* Next digit */
	    range_value = (range_value * 10) + (ch - '0');
        }
        ch = uart_get_char(1);
    }

/*
    if (mdetect) {
	if (!motionDetected)
	    ESP_LOGI(RX_TASK_TAG, "Motion detected: On");
	motionDetected = mdetect;
    }
*/
}

static void normal_off(void)
{
    range_value = 0;
    average_value = 0;
    measurement_count = 0;
    ld2420_Distance = 0;

    if (motionDetected)
	ESP_LOGI(RX_TASK_TAG, "Motion detected: Off");

    motionDetected = 0;
    timestamps[TS_STALL] = timestamps[TS_STALL11] = clock();
}

static void command_handler(void)
{
    int len = uart_get_char(2);
    int status, param;

    if (len >= 2) {
        int cmd = uart_get_char(2);
//        ESP_LOGI(RX_TASK_TAG, "Cmd: %02x, l=%02x, idx=%d", cmd, len, abd_param_index);

        switch (cmd) {
            case config_read_resp:
                if (len >= 8 && abd_param_index >= 0) {
                    status = uart_get_char(2);
                    param =  uart_get_char(4);
                    ESP_LOGI(RX_TASK_TAG, "Param read: %d, %d", status, param);
                    if (0 == status) { /* If successfully read mark parameter for update if its value is not current. */
                        abd_parameters[abd_param_index].need_update =
                            abd_parameters[abd_param_index].value != param;
                    }
                }
//                timestamps[TS_PARAM] = clock();
                abd_param_index = -1;
                break;
            case config_write_resp:
                if (len >= 4 && abd_param_index >= 0) {
                    status = uart_get_char(2);
                    ESP_LOGI(RX_TASK_TAG, "Param write: %d", status);
                    if (0 == status) {
                        abd_parameters[abd_param_index].need_update = 0;
                    } else {
                        param_write_status = (uint8_t)status;
                    }
                }
                abd_param_index = -1;
//                timestamps[TS_PARAM] = clock();
                break;
            default:
                break;

        }

    }
}

struct {
    uint32_t pattern_word;
    void (*handler)(void);
} ld2420_handlers[3] = {
    {/* Normal output: Range dd */
        .pattern_word = ('n' << 24) + ('g' << 16) + ('e' << 8) + ' ',
        .handler = normal_range,
    },

    {/* No motion detected: OFF */
        .pattern_word = ('O' << 24) + ('F' << 16) + ('F' << 8) + '\r',
        .handler = normal_off,
    },

    {/* Command header: 0xFDFCFBFA */
        .pattern_word = 0xFDFCFBFA,
        .handler = command_handler,
    },

};

void check_pattern(uint32_t pattern)
{
    int i;
//    int idx = -1;
    for (i = 0; i < sizeof(ld2420_handlers)/sizeof(ld2420_handlers[0]); i++) {
        if (pattern == ld2420_handlers[i].pattern_word) {
            ld2420_handlers[i].handler();
//            idx = i;
        }
    }
//    ESP_LOGI(RX_TASK_TAG, "Pattern: %08lx, %08lx, idx=%d", pattern, ld2420_handlers[0].pattern_word, idx);
}

//static const uint32_t cmd_pattern = 0xFAFBFCFD;

static const uint16_t hdr1 = 0xFCFD;
static const uint16_t hdr2 = 0xFAFB;

static const uint16_t sfx1 = 0x0304;
static const uint16_t sfx2 = 0x0102;

static const uint16_t write_cmd = 0x7;
static const uint16_t read_cmd = 0x8;

static uint16_t command_buffer[MAX_TX_BUF_WORDS];



/* */
void ld2420_param_update(int param_idx, uint64_t clock_now)
{
/* Must finish one request before sending another */
    if (abd_param_index >= 0)
        return;

    abd_param_index = param_idx;
    uint32_t value = abd_parameters[abd_param_index].value;

    command_buffer[0] = hdr1;
    command_buffer[1] = hdr2;
    command_buffer[2] = 8;         /* data length */
    command_buffer[3] = write_cmd; /* command */
    command_buffer[4] = param_idx;
    command_buffer[5] = value & 0xffff;
    command_buffer[6] = value >> 16;

    command_buffer[7] = sfx1;
    command_buffer[8] = sfx2;
    uart_write_bytes(UART_NUM_1, command_buffer, 9*2);

    timestamps[TS_PARAM] = clock_now;
}

/* Check the sensor timeout setting and record current time to monitor sensor response time.
 */
void ld2420_param_check(int param_idx, uint64_t clock_now)
{
/* Must finish one request before sending another */
    if (abd_param_index >= 0)
        return;

    abd_param_index = param_idx;

    command_buffer[0] = hdr1;
    command_buffer[1] = hdr2;
    command_buffer[2] = 4;        /* data length */
    command_buffer[3] = read_cmd; /* command */
    command_buffer[4] = param_idx;/* address */
    command_buffer[5] = sfx1;
    command_buffer[6] = sfx2;
    uart_write_bytes(UART_NUM_1, command_buffer, 7*2);

    timestamps[TS_PARAM] = clock_now;
}

/* Power-cycle the sensor */
static void ld2420_reset(uint64_t clock_now)
{
    gpio_set_level(LD2420_POWER_PIN, !POWER_PIN_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LD2420_POWER_PIN, POWER_PIN_ON_LEVEL);

/* Reset the timestamps to avoid restart loops */
    int i;
    for (i = 0; i < TS_count; i++)
//        if (i != TS_DETECT)
            timestamps[i] = clock_now + timer_resolution;

    abd_parameters[4].need_update = 1;
}

static void check_comm_timeouts(uint64_t clock_now)
{
    int i;

//    int mdetect = 1;
//    timeouts[TS_DETECT] = (uint64_t)timer_resolution * motionTimeout;

    for (i = TS_PARAM; i < TS_count; i++) {
        if (i == TS_PARAM && abd_param_index < 0)
            continue;

        if (clock_now < timestamps[i])
            continue;

        uint64_t delta = clock_now - timestamps[i];
        if (delta > timeouts[i]) {

//            if (i == TS_DETECT) {
//                mdetect = 0;
//                continue;
//            }

            ld2420_reset(clock_now);
            abd_param_index = -1;
//            ESP_LOGI(RX_TASK_TAG, "Timout %d = %lu: reset. Time: %lu, ", i, (uint32_t)delta, (uint32_t)clock_now);
            ESP_LOGI(RX_TASK_TAG, "Timout %d clock_now: %lu, clock: %lu, timestamp = %lu", i, (uint32_t)clock_now, (uint32_t)clock(), (uint32_t)timestamps[i]);
            break;
        }
    }

//    if (motionDetected != mdetect)
//        ESP_LOGI(RX_TASK_TAG, "Motion detected: %d", mdetect);

//    motionDetected = mdetect;
}


void ld2420_task(void *arg)
{
    init_gpio();
    init_uart();

/* This delay is needed for UART to become active */
    vTaskDelay(pdMS_TO_TICKS(2000));

    init_timer(&timer, timer_resolution);

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    int ch = 0;
    uint32_t detect = 0;
    uint64_t clock_now = 0;
    ld2420_reset(clock_now);

    while (1) {
        ch = uart_get_char(1);
        if (ch >= 0 && ch < 256) {
            detect = (detect << 8) | ch;
            switch (ch) { /* Check last bytes of the registered patterns to speed up processing */
                case ' ':
                case '\r':
                case 0xFA:
                    check_pattern(detect);
                    break;
                default:
                    break;
            }

        }

        clock_now = clock();
        uint64_t delta = clock_now - param_time;
        if (delta > param_update_time) {
//            ESP_LOGI(RX_TASK_TAG, "Check: %lu, idx=%d", (uint32_t)(delta), abd_param_index);

     /* watch only the timeout setting (index 4) for now */
            if (abd_parameters[4].need_update) {
                ld2420_param_update(4, clock_now);
            } else {
                ld2420_param_check(4, clock_now);
            }
            param_time = clock_now;
        }

        check_comm_timeouts(clock_now);
    }
//            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
}

