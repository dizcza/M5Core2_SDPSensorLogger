/*
 * board.h
 *
 *  Created on: Dec 8, 2021
 *      Author: Danylo Ulianych
 */

#ifndef MAIN_INCLUDE_BOARD_H_
#define MAIN_INCLUDE_BOARD_H_


#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>

#ifndef BSP_LOGI
#define BSP_LOGI ESP_LOGI
#endif

#ifndef BSP_LOGW
#define BSP_LOGW ESP_LOGW
#endif

#ifndef BSP_LOGE
#define BSP_LOGE ESP_LOGE
#endif


typedef struct {
	int miso;
	int mosi;
	int clk;
	int cs;
} sd_spi_gpio_t;


typedef struct {
	uint8_t i2c_addr;
	int sda;
	int scl;
	int irq;
} sdp_gpio_t;


typedef struct {
	int sda;
	int scl;
} bmp_gpio_t;


typedef struct {
   uint64_t mac;
   sd_spi_gpio_t spi_gpio;
   sdp_gpio_t sdp_gpio;
   bmp_gpio_t bmp_gpio;
   void (*led_init)();
   void (*led_set_error)();
   void (*led_clear_error)();
} Board_t;


void board_init();
const Board_t* board_get();


#ifdef __cplusplus
}
#endif


#endif /* MAIN_INCLUDE_BOARD_H_ */
