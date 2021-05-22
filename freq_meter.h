/*
 * freq_cnt.h
 *
 *  Created on: 6 maj 2020
 *      Author: kurzawa.p
 */

#ifndef FREQ_CNT_H
#define FREQ_CNT_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "driver/rmt.h"
#include "driver/pcnt.h"
#include "esp_err.h"

/*********************
 *      DEFINES
 *********************/
#define FM_PIN_MAX (2)
#define FM_MEASURE_CONTROL_PIN CONFIG_FM_CONTROL_PIN

#define PCNT_UNIT (PCNT_UNIT_0)
#define PCNT_CHANNEL (PCNT_CHANNEL_0)
#define RMT_CHANNEL (RMT_CHANNEL_0)
#define RMT_CLK_DIV (20) // results in 2us steps (80MHz / 160 = 0.5 MHz
//#define RMT_CLK_DIV 20 // results in 0.25us steps (80MHz / 20 = 4 MHz
//#define RMT_CLK_DIV       1     // results in 25ns steps (80MHz / 2 / 1 = 40 MHz)

// The counter is signed 16-bit, so maximum positive value is 32767
// The filter is unsigned 10-bit, maximum value is 1023. Use full period of maximum frequency.
// For higher expected frequencies, the sample period and filter must be reduced.

// suitable up to 163,835 Hz
#define WINDOW_DURATION  (CONFIG_FM_SAMPLE_TIME / 1000.0F)// (0.05F) //0.5 // seconds
#define FILTER_LENGTH 1      // APB @ 80MHz, limits to < 655,738 Hz

#if (FILTER_LENGTH > 1023)
#error "The filter is unsigned 10-bit, maximum value is 1023."
#endif

/**********************
 *      TYPEDEFS
 **********************/
typedef void *fm_handle_t;
typedef void (* measure_finish_callback_t)(uint8_t input, double freq_hz);

/**********************
 * GLOBAL PROTOTYPES
 **********************/
fm_handle_t *freq_meter_create(rmt_channel_t rmt_channel, uint8_t rmt_clk_div, float sampling_window_seconds, uint8_t pin_ctrl);
esp_err_t freq_meter_destroy(fm_handle_t const fm_handle);
esp_err_t freq_meter_add_pin(fm_handle_t const fm_handle, uint8_t io_num, pcnt_unit_t pcnt_unit, pcnt_channel_t pcnt_channel, uint16_t filter_length);
esp_err_t freq_meter_measure_start(fm_handle_t const fm_handle);
esp_err_t freq_meter_measure_finish_add_callback(fm_handle_t const fm_handle, measure_finish_callback_t cb);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* COMPONENTS_FREQ_CNT_FREQ_CNT_H_ */
