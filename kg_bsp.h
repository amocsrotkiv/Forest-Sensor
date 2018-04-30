#ifndef K_BSP_H__
#define K_BSP_H__

#include "nrf_gpio.h"
#include "nrf_delay.h"

#define		PIN_NO_CONN_1					15
#define		PIN_NO_CONN_2					16

#define		PIN_LED_1						23
#define		PIN_LED_2						22
#define		PIN_BUTTON					19
#define		PIN_NFC_2						10

#define		PIN_LORA_EN					13
#define		PIN_GNSS_EN					2

//#define		PIN_BAT_LVL_1_2			20  // v1
#define		PIN_BAT_LVL_1_2			4
#define		PIN_CH_IND					24
#define		PIN_POW_EN					17

#define		PIN_LORA_SPI_CLK			5
//#define		PIN_LORA_SPI_CS			4		// v1
#define 		PIN_LORA_SPI_CS			8
#define		PIN_LORA_SPI_MISO			3
#define		PIN_LORA_SPI_MOSI			6

#define		PIN_LORA_RST				14

#define		PIN_LORA_IO_0				11
#define		PIN_LORA_IO_1				9
#define		PIN_LORA_IO_2				7


#define		PIN_GNSS_SPI_CLK			28
#define		PIN_GNSS_SPI_CS			30
#define		PIN_GNSS_SPI_MISO		29
#define		PIN_GNSS_SPI_MOSI		27

#define		PIN_GNSS_INT					26
#define		PIN_GNSS_TPS				25
#define		PIN_GNSS_RST				31

void initialize_peripherals(void);

void prepare_low_power_mode(void);

void enable_GNSS(bool state);

void enable_LORA(bool state);

void reset_LORA(void);

void reset_GNSS(void);

void switch_off_device(void);


#endif
