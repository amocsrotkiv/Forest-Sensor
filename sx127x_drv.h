/* Copyright (c) 2016 OMTLAB Research Ltd. All Rights Reserved.
 *
 */

#ifndef SX127X_DRV_H
#define SX127X_DRV_H

#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf_log.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"


#define CFG_sx1276_radio
#define CFG_eu868

#include "lmic.h"
#include "kg_bsp.h"
#include "nrf_log.h"
//#include "uart_log.h"

/* Copyright (c) 2016 OMTLAB Research Ltd. All Rights Reserved.
 *
 */

void sx127x_init(uint32_t pin_mosi, uint32_t pin_miso, uint32_t pin_sck, uint32_t pin_cs, uint32_t rst_pin,void (*eventHandler)(uint8_t *rxPkt));
void sx127x_sendPkt(uint8_t *pktData);
void sx127x_startRx();
int8_t sx127x_getPktRSSI();
int8_t sx127x_getPktSNR();



#endif // NRF_DRV_CONFIG_H

