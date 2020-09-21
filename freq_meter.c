/*
 * freq_cnt.c
 *
 *  Created on: 6 maj 2020
 *      Author: kurzawa.p
 */

/*********************
 *      INCLUDES
 *********************/
#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freq_meter.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
	gpio_num_t pcnt_gpio;     ///< count events on this GPIO
	pcnt_unit_t pcnt_unit; ///< PCNT unit to use for counting
	pcnt_channel_t pcnt_channel; ///< PCNT channel to use for counting
	uint16_t pcnt_filter_length; ///< counter filter length in APB cycles
} fm_measure_pin_cfg_t;

typedef struct
{
	fm_measure_pin_cfg_t *measure_pin_cfg[FM_PIN_MAX];
	rmt_channel_t rmt_channel;   ///< The RMT channel to use
	uint8_t rmt_clk_div; ///< RMT pulse length, as a divider of the APB clock
	float sampling_window_in_s; ///< sample window length (in seconds)
	rmt_item32_t *rmt_items_buff;
	uint32_t rmt_items_cnt;
	gpio_num_t pin_control;
	void (*frequency_update_cb)(uint8_t input, double freq_hz); ///< called each time a frequency is determined
	SemaphoreHandle_t polling_semaphore_handle;
} fm_cfg_t;

typedef enum
{
	FM_DESTROY,
	FM_SEM_CREATE_FAIL,
	FM_RMT_INIT_FAIL,
	FM_RMT_INTR_INIT_FAIL,
	FM_CREATE_BUFF_FAIL,
	FM_FILL_BUFF_FAIL,
} fm_rollback_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static esp_err_t freq_meter_rollback(fm_rollback_t step, fm_cfg_t *cfg);
static esp_err_t init_rmt(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div);
static void init_pcnt(uint8_t pulse_gpio, uint8_t ctrl_gpio, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t pcnt_filter_length);
static uint8_t create_rmt_buffer(rmt_item32_t **items, uint8_t rmt_clk_div, double sampling_window_in_s);
static uint16_t fill_rmt_buffer(rmt_item32_t *items, uint8_t rmt_clk_div, double sampling_window_in_s);
static void freq_meter_measure_end(rmt_channel_t channel, void *arg);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "freq_meter";

/**********************
 *      MACROS
 **********************/
#define CHECK(a, ret_val, str, ...)                                               \
      if (!(a))                                                                   \
      {                                                                           \
            ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            return (ret_val);                                                     \
      }

#define IOT_CHECK(tag, a, ret_val)                                     \
    if (!(a))                                                          \
    {                                                                  \
        ESP_LOGE(tag, "%s:%u (%s)", __FILE__, __LINE__, __FUNCTION__); \
        return (ret_val);                                              \
    }
#define ERR_ASSERT(tag, param) IOT_CHECK(tag, (param) == ESP_OK, ESP_FAIL)
#define POINT_ASSERT(tag, param) IOT_CHECK(tag, (param) != NULL, NULL)

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
fm_handle_t* freq_meter_create(rmt_channel_t rmt_channel, uint8_t rmt_clk_div, float sampling_window_in_s, uint8_t pin_ctrl)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	CHECK(rmt_channel < RMT_CHANNEL_MAX, NULL, "RMT CHANNEL ERR");
	CHECK(GPIO_IS_VALID_OUTPUT_GPIO(pin_ctrl), NULL, "RMT GPIO ERROR");
	CHECK(rmt_clk_div > 0, NULL, "RMT DRIVER ERR");

	fm_cfg_t *fm_cfg_p = (fm_cfg_t*) calloc(1, sizeof(fm_cfg_t));
	CHECK((fm_cfg_p != NULL), NULL, "FM CONFIG ALLOC ERROR");

	fm_cfg_p->polling_semaphore_handle = xSemaphoreCreateBinary();

	if (fm_cfg_p->polling_semaphore_handle == NULL)
	{
		freq_meter_rollback(FM_SEM_CREATE_FAIL, fm_cfg_p);
		return NULL;
	}

	vQueueAddToRegistry(fm_cfg_p->polling_semaphore_handle, "fm_semaphore");


	ESP_LOGD(TAG, "rmt_channel %d|rmt_clk_div %d|sampling_window_in_s %.2f|pin_ctrl %d", rmt_channel, rmt_clk_div, sampling_window_in_s, pin_ctrl);

	fm_cfg_p->rmt_channel = rmt_channel;
	fm_cfg_p->rmt_clk_div = rmt_clk_div;
	fm_cfg_p->pin_control = pin_ctrl;
	fm_cfg_p->sampling_window_in_s = sampling_window_in_s;

	esp_err_t err = init_rmt(pin_ctrl, rmt_channel, rmt_clk_div);
	if (err != ESP_OK)
	{
		freq_meter_rollback(FM_RMT_INIT_FAIL, fm_cfg_p);
		return NULL;
	}

	rmt_register_tx_end_callback(freq_meter_measure_end, (void*) fm_cfg_p);
	err = rmt_set_tx_intr_en(rmt_channel, true);
	if (err != ESP_OK)
	{
		freq_meter_rollback(FM_RMT_INTR_INIT_FAIL, fm_cfg_p);
		return NULL;
	}

	for (uint8_t i = 0; i < FM_PIN_MAX; i++)
	{
		fm_cfg_p->measure_pin_cfg[i] = NULL;
	}

	rmt_item32_t *rmt_items_buff = NULL;

	uint8_t req_blocks = create_rmt_buffer(&rmt_items_buff, rmt_clk_div, sampling_window_in_s);
	if (req_blocks == 0 || rmt_items_buff == NULL)
	{
		freq_meter_rollback(FM_CREATE_BUFF_FAIL, fm_cfg_p);
		return NULL;
	}

	fm_cfg_p->rmt_items_buff = rmt_items_buff;

	uint16_t rmt_items_cnt = fill_rmt_buffer(rmt_items_buff, rmt_clk_div, sampling_window_in_s);
	if (rmt_items_cnt > req_blocks * RMT_MEM_ITEM_NUM)
	{
		freq_meter_rollback(FM_FILL_BUFF_FAIL, fm_cfg_p);
		return NULL;
	}

	fm_cfg_p->rmt_items_cnt = rmt_items_cnt;

	xSemaphoreGive(fm_cfg_p->polling_semaphore_handle);

	return (fm_handle_t*) fm_cfg_p;
}

/**
 *
 * @param fm_handle
 * @return
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_OK Success
 */
esp_err_t freq_meter_destroy(fm_handle_t const fm_handle)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	IOT_CHECK(TAG, fm_handle != NULL, ESP_ERR_INVALID_ARG);

	fm_cfg_t *fm_cfg = (fm_cfg_t*) fm_handle;

	esp_err_t err = freq_meter_rollback(FM_DESTROY, fm_cfg);
	CHECK((err == ESP_OK), ESP_FAIL, "FM DESTROY FAILED");
	return err;
}

/**
 *
 * @param fm_handle
 * @param io_num
 * @param pcnt_unit
 * @param pcnt_channel
 * @param filter_length
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM calloc memory error
 *     - ESP_ERR_NOT_FOUND measure input slots full
 *     - ESP_OK Success
 */
esp_err_t freq_meter_pin_add(fm_handle_t const fm_handle, uint8_t input_io_num, pcnt_unit_t pcnt_unit, pcnt_channel_t pcnt_channel, uint16_t filter_length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	IOT_CHECK(TAG, fm_handle != NULL, ESP_ERR_INVALID_ARG);
	IOT_CHECK(TAG, pcnt_unit < PCNT_UNIT_MAX, ESP_ERR_INVALID_ARG);
	IOT_CHECK(TAG, pcnt_channel < PCNT_CHANNEL_MAX, ESP_ERR_INVALID_ARG);

	ESP_LOGD(TAG, "input_io_num %d|pcnt_unit %d|pcnt_channel %d|filter_length %d", input_io_num, pcnt_unit, pcnt_channel, filter_length);

	fm_cfg_t *fm_cfg = (fm_cfg_t*) fm_handle;
	xSemaphoreTake(fm_cfg->polling_semaphore_handle, portMAX_DELAY);

	fm_measure_pin_cfg_t *fm_measure_pin_cfg = (fm_measure_pin_cfg_t*) calloc(1, sizeof(fm_measure_pin_cfg_t));
	IOT_CHECK(TAG, fm_measure_pin_cfg != NULL, ESP_ERR_NO_MEM);

	init_pcnt(input_io_num, fm_cfg->pin_control, pcnt_unit, pcnt_channel, filter_length);
	ESP_LOGD(TAG, "Filter freq %.1f Hz", (double)(APB_CLK_FREQ / filter_length / 2.0));

	fm_measure_pin_cfg->pcnt_gpio = input_io_num;
	fm_measure_pin_cfg->pcnt_unit = pcnt_unit;
	fm_measure_pin_cfg->pcnt_channel = pcnt_channel;
	fm_measure_pin_cfg->pcnt_filter_length = filter_length;

	for (int i = 0; i < FM_PIN_MAX; i++)
	{
		if (fm_cfg->measure_pin_cfg[i] == NULL)
		{
			fm_cfg->measure_pin_cfg[i] = fm_measure_pin_cfg;
			break;
		}
		IOT_CHECK(TAG, i < FM_PIN_MAX, ESP_ERR_NOT_FOUND);
	}
	xSemaphoreGive(fm_cfg->polling_semaphore_handle);
	return ESP_OK;
}

esp_err_t freq_meter_measure_finish_add_cb(fm_handle_t const fm_handle, measure_finish_cb_t cb)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	IOT_CHECK(TAG, fm_handle != NULL, ESP_ERR_INVALID_ARG);
	IOT_CHECK(TAG, cb != NULL, ESP_ERR_INVALID_ARG);

	fm_cfg_t *fm_cfg = (fm_cfg_t*) fm_handle;
	fm_cfg->frequency_update_cb = cb;
	return ESP_OK;
}

esp_err_t freq_meter_measure_start(fm_handle_t const fm_handle)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	IOT_CHECK(TAG, fm_handle != NULL, ESP_ERR_INVALID_ARG);

	fm_cfg_t *fm_cfg = (fm_cfg_t*) fm_handle;

	IOT_CHECK(TAG, fm_cfg->rmt_channel < RMT_CHANNEL_MAX, ESP_ERR_INVALID_ARG);
	IOT_CHECK(TAG, fm_cfg->rmt_items_buff != NULL, ESP_ERR_INVALID_ARG);
	IOT_CHECK(TAG, fm_cfg->rmt_items_cnt > 0, ESP_ERR_INVALID_ARG);

	xSemaphoreTake(fm_cfg->polling_semaphore_handle, portMAX_DELAY);

	for (int i = 0; i < FM_PIN_MAX; i++)
	{
		if (fm_cfg->measure_pin_cfg[i] != NULL)
		{
			// clear counter
			ESP_ERROR_CHECK(pcnt_counter_clear(fm_cfg->measure_pin_cfg[i]->pcnt_unit));
		}
	}
	// start sampling window
	ESP_ERROR_CHECK(rmt_write_items(fm_cfg->rmt_channel, fm_cfg->rmt_items_buff, fm_cfg->rmt_items_cnt, false));
	return ESP_OK;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
static esp_err_t freq_meter_rollback(fm_rollback_t step, fm_cfg_t *cfg)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	switch (step)
	{
		case FM_DESTROY:
			xSemaphoreTake(cfg->polling_semaphore_handle, portMAX_DELAY);
			for (uint8_t i = 0; i < FM_PIN_MAX; i++)
			{
				if (cfg->measure_pin_cfg[i] != NULL)
				{
					free(cfg->measure_pin_cfg[i]);
				}
			}
			/* no break */
		case FM_FILL_BUFF_FAIL:
			if (step == FM_FILL_BUFF_FAIL)
				ESP_LOGE(TAG, "FM FILL BUFF FAIL");
			free(cfg->rmt_items_buff);
			/* no break */
		case FM_CREATE_BUFF_FAIL:
			if (step == FM_CREATE_BUFF_FAIL)
				ESP_LOGE(TAG, "FM CREATE BUFF FAIL");
			rmt_set_tx_intr_en(cfg->rmt_channel, false);
			/* no break */
		case FM_RMT_INTR_INIT_FAIL:
			if (step == FM_RMT_INTR_INIT_FAIL)
				ESP_LOGE(TAG, "RMT INIT INTERUPT FAIL");
			rmt_register_tx_end_callback(NULL, NULL);
			rmt_driver_uninstall(cfg->rmt_channel);
			/* no break */
		case FM_RMT_INIT_FAIL:
			if (step == FM_RMT_INIT_FAIL)
				ESP_LOGE(TAG, "RMT INIT FAIL");
			vQueueUnregisterQueue(cfg->polling_semaphore_handle);
			vSemaphoreDelete(cfg->polling_semaphore_handle);
			/* no break */
		case FM_SEM_CREATE_FAIL:
			if (step == FM_SEM_CREATE_FAIL)
				ESP_LOGE(TAG, "FM SEMAPHORE CREATE FAIL");
			free(cfg);
			/* no break */
		default:
			return ESP_OK;
			break;
	}
	return ESP_FAIL;
}

static void IRAM_ATTR freq_meter_measure_end(rmt_channel_t channel, void *arg)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	fm_cfg_t *fm_cfg = (fm_cfg_t*) arg;

	// read counter
	for (uint8_t input = 0; input < FM_PIN_MAX; input++)
	{
		if (fm_cfg->measure_pin_cfg[input] == NULL)
		{
			continue;
		}

		int16_t count = 0;
		ESP_ERROR_CHECK(pcnt_get_counter_value(fm_cfg->measure_pin_cfg[input]->pcnt_unit, &count));
		xSemaphoreGiveFromISR(fm_cfg->polling_semaphore_handle, &pxHigherPriorityTaskWoken);
		double frequency_hz = abs(count / 2.0 / fm_cfg->sampling_window_in_s);

		if (fm_cfg->frequency_update_cb)
		{
			fm_cfg->frequency_update_cb(input, frequency_hz);
		}
	}

	if (pxHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR();
	}
}

static esp_err_t init_rmt(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	rmt_config_t rmt_tx =
		{
			.rmt_mode = RMT_MODE_TX,
			.channel = channel,
			.gpio_num = tx_gpio,
			.mem_block_num = 1, // single block
			.clk_div = clk_div,
			.tx_config.loop_en = false,
			.tx_config.carrier_en = false,
			.tx_config.idle_level = RMT_IDLE_LEVEL_LOW,
			.tx_config.idle_output_en = true, };
	esp_err_t err = rmt_config(&rmt_tx);
	CHECK((err == ESP_OK), ESP_FAIL, "Fail rmt_config");

	err = rmt_driver_install(channel, 0, 0);
	CHECK((err == ESP_OK), ESP_FAIL, "Fail rmt_driver_install");

	return ESP_OK;
}

static void init_pcnt(uint8_t pulse_gpio, uint8_t ctrl_gpio, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t pcnt_filter_length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	// set up counter
	pcnt_config_t pcnt_config =
		{
			.pulse_gpio_num = pulse_gpio,
			.ctrl_gpio_num = ctrl_gpio,
			.lctrl_mode = PCNT_MODE_DISABLE,
			.hctrl_mode = PCNT_MODE_KEEP,
			.pos_mode = PCNT_COUNT_INC, // count both rising and falling edges
			.neg_mode = PCNT_COUNT_INC,
			.counter_h_lim = 0,
			.counter_l_lim = 0,
			.unit = unit,
			.channel = channel, };

	ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

	// set the GPIO back to high-impedance, as pcnt_unit_config sets it as pull-up
	//ESP_ERROR_CHECK(gpio_set_pull_mode(pulse_gpio, GPIO_FLOATING));

	// enable counter filter - at 80MHz APB CLK, 1000 pulses is max 80,000 Hz, so ignore pulses less than 12.5 us.
	ESP_ERROR_CHECK(pcnt_set_filter_value(unit, pcnt_filter_length));
	ESP_ERROR_CHECK(pcnt_filter_enable(unit));
}

/**
 *
 * @param items
 * @param rmt_clk_div
 * @param sampling_window_in_s
 * @return RMT blocks require
 */
static uint8_t create_rmt_buffer(rmt_item32_t **items, uint8_t rmt_clk_div, double sampling_window_in_s)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	const double rmt_period = (double) (rmt_clk_div) / APB_CLK_FREQ; // czas trwania jednego impulsu [s]
	const double item_duration = (2.0 * 32767.0) * rmt_period; // czas trwania jednego itemu
	uint8_t req_blocks = (uint8_t) ceil((sampling_window_in_s / item_duration) / RMT_MEM_ITEM_NUM); //8 bloków po 64 itemy
	ESP_LOGD(TAG, "Req blocks %d", req_blocks);
	CHECK((req_blocks <= 8), 0, "TOO LONG SAMPLE WINDOW");
	//okno samplingu / licznik countera / 2 (zliczamy oba zbocza)
	ESP_LOGD(TAG, "Maximum freq %.1f Hz", (float)(1.0 / (sampling_window_in_s / (float)(INT16_MAX - INT16_MIN) / 2.0)));
	ESP_LOGD(TAG, "Minimum freq %f Hz", (float )sampling_window_in_s / 10);
	size_t items_size_in_byte = RMT_MEM_ITEM_NUM * req_blocks * sizeof(rmt_item32_t);
	ESP_LOGD(TAG, "Items size %d bytes", items_size_in_byte);

	*items = calloc(RMT_MEM_ITEM_NUM * req_blocks, sizeof(rmt_item32_t));
	CHECK((*items != NULL), 0, "RMT BUFFER ALLOC ERROR");

	return req_blocks;
}

static uint16_t fill_rmt_buffer(rmt_item32_t *items, uint8_t rmt_clk_div, double sampling_window_in_s)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	IOT_CHECK(TAG, items != NULL, ESP_ERR_INVALID_ARG);

	const double rmt_period = (double) (rmt_clk_div) / APB_CLK_FREQ; // czas trwania jednego impulsu [s]
	uint16_t num_items = 0;
	// enable counter for exactly x seconds:
	uint32_t total_duration = (uint32_t) (sampling_window_in_s / rmt_period);
	ESP_LOGD(TAG, "total_duration %f seconds = %d * %g seconds", sampling_window_in_s, total_duration, rmt_period);
	//memset(items, 0, sizeof(items));
	uint16_t *p_items = (uint16_t*) items;
	// max duration per item is 2^15-1 = 32767
	while (total_duration > 0)
	{
		uint32_t duration = total_duration > 32767 ? 32767 : total_duration;
		p_items[num_items] = (1 << 15) | duration;
		total_duration -= duration;
		++num_items;
	}

	return (num_items + 1) / 2;
}
