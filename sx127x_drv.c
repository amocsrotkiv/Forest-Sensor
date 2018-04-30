/* Copyright (c) 2016 OMTLAB Research Ltd. All Rights Reserved.
 * SX127X radio driver from IBM's LMIC 1.6 LoRaWAN implementation
 *
 */
/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"


#include "app_util_platform.h"

#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define CFG_sx1276_radio
#define CFG_eu868

#include "sx127x_drv.h"
#include "lmic.h"


static uint32_t			 lora_rst_pin;

static uint8_t       spi_tx_buf[30];    /**< TX buffer. */
static uint8_t       spi_rx_buf[30];    /**< RX buffer. */
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
//static uint8_t		   spiReady;

static const nrf_drv_spi_t m_spi_master_1 = NRF_DRV_SPI_INSTANCE(1);

// ---------------------------------------- 
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F 
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11 
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12 
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13 
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14 
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15 
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16 
#define LORARegRxpacketCntValueLsb                 0x17 
#define LORARegModemStat                           0x18 
#define LORARegPktSnrValue                         0x19 
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A 
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B 
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C 
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D 
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E 
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F 
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20 
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21 
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22 
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23 
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24 
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
#define RegTcxo                                    0x4B // common
#define RegPaDac                                   0x5A // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
// #define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK  0x00
#define SX1272_MC2_SF7  0x70
#define SX1272_MC2_SF8  0x80
#define SX1272_MC2_SF9  0x90
#define SX1272_MC2_SF10 0xA0
#define SX1272_MC2_SF11 0xB0
#define SX1272_MC2_SF12 0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125  0x00
#define SX1272_MC1_BW_250  0x40
#define SX1272_MC1_BW_500  0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5 0x08
#define SX1272_MC1_CR_4_6 0x10
#define SX1272_MC1_CR_4_7 0x18
#define SX1272_MC1_CR_4_8 0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON 0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON        0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN 0x00


// sx1276 RegModemConfig1
#define SX1276_MC1_BW_10                 0x10
#define SX1276_MC1_BW_20                 0x30
#define SX1276_MC1_BW_40                 0x50
#define SX1276_MC1_BW_60                 0x60
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01 
                                                    
// sx1276 RegModemConfig2          
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3          
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef CFG_sx1276_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif CFG_sx1272_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif



// ---------------------------------------- 
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06 
#define OPMODE_CAD       0x07 

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

#define MAP_DIO0_FSK_READY     0x00  // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP       0x30  // --11----
#define MAP_DIO2_FSK_TXNOP     0x04  // ----01--
#define MAP_DIO2_FSK_TIMEOUT   0x08  // ----10--


// FSK IMAGECAL defines
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default


// RADIO STATE
// (initialized by radio_init(), used by radio_rand1())
static u1_t randbuf[16];

static u1_t lora_txbuf[30];
static u1_t lora_rxbuf[30];
static u1_t lora_dataLen;
static s1_t lora_snr;
static s1_t lora_rssi;



// profile 1
//static u4_t sx127x_freq = EU868_F6;

//static u1_t sx127x_syncWord = 0x6D;
//static u1_t sx127x_bw = SX1276_MC1_BW_125;
//static u1_t sx127x_coding = SX1276_MC1_CR_4_5;
//static u1_t sx127x_spreadingFactor = SX1272_MC2_SF12;
//static u1_t sx127x_power = 15;
//static u1_t sx127x_pktSize = 15;


// profile 2

//static u4_t sx127x_freq = EU868_F6;

//static u1_t sx127x_syncWord = 0x59;
//static u1_t sx127x_bw = SX1276_MC1_BW_20;
//static u1_t sx127x_coding = SX1276_MC1_CR_4_5;
//static u1_t sx127x_spreadingFactor = SX1272_MC2_SF12;
//static u1_t sx127x_power = 15;
//static u1_t sx127x_pktSize = 15;
//static u1_t sx127x_preambleLen = 20;

// profile 3
/*
static u4_t sx127x_freq = EU868_F6;

static u1_t sx127x_syncWord = 0x6D;
static u1_t sx127x_bw = SX1276_MC1_BW_40;
static u1_t sx127x_coding = SX1276_MC1_CR_4_5;
static u1_t sx127x_spreadingFactor = SX1276_MC2_SF10;
static u1_t sx127x_power = 15;
static u1_t sx127x_pktSize = 15;
*/

// profile 4

static u4_t sx127x_freq = EU868_F6;

static u1_t sx127x_syncWord = 0x6D;
static u1_t sx127x_bw = SX1276_MC1_BW_60;
static u1_t sx127x_coding = SX1276_MC1_CR_4_5;
static u1_t sx127x_spreadingFactor = SX1272_MC2_SF12;
static u1_t sx127x_power = 15;
static u1_t sx127x_pktSize = 15;
static u1_t sx127x_preambleLen = 8;

static void (*sx127xEventHandler)(uint8_t *rxPkt);

#define MINRX_SYMS 5


#ifdef CFG_sx1276_radio
//#define LNA_RX_GAIN (0x20|0x1)
#define LNA_RX_GAIN (0x20|0x3)
#elif CFG_sx1272_radio
#define LNA_RX_GAIN (0x20|0x03)
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif

void hal_pin_rxtx(uint8_t mode)
{
	// since with have on onboard RX/TX switch, do nothing
}

void hal_enableIRQs()
{
	
}

void hal_disableIRQs()
{
	
}	
	
void hal_pin_rst(uint8_t mode)
{
	switch (mode) {
		case 0:
			nrf_gpio_cfg_output(lora_rst_pin);
			nrf_gpio_pin_clear(lora_rst_pin);
		break;
		case 1:
			nrf_gpio_cfg_output(lora_rst_pin);
			nrf_gpio_pin_set(lora_rst_pin);
		break;
		case 2:
			nrf_gpio_cfg_input(lora_rst_pin,NRF_GPIO_PIN_PULLUP);
		break;
	}
}


static void writeReg (u1_t addr, u1_t data ) {
		spi_tx_buf[0] = addr | 0x80;
		spi_tx_buf[1] = data;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi_master_0, spi_tx_buf, 2, spi_rx_buf, 2));
}

static u1_t readReg (u1_t addr) {
		spi_tx_buf[0] = addr & 0x7F;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi_master_0, spi_tx_buf, 1, spi_rx_buf, 2));
    return spi_rx_buf[1];
}

static void writeBuf (u1_t addr, xref2u1_t buf, u1_t len) {
		spi_tx_buf[0] = addr | 0x80;
    for (u1_t i=0; i<len; i++) {
        spi_tx_buf[1+i] = buf[i];
    }
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi_master_0, spi_tx_buf, 1+len, spi_rx_buf, 1+len));
}

static void readBuf (u1_t addr, xref2u1_t buf, u1_t len) {
		memset(spi_tx_buf,0x00,16);
		spi_tx_buf[0] = addr & 0x7F;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi_master_0, spi_tx_buf, 1+len, spi_rx_buf, 1+len));
    for (u1_t i=0; i<len; i++) {
        buf[i] = spi_rx_buf[1+i];
    }
}




static void opmode (u1_t mode) {
    writeReg(RegOpMode, (readReg(RegOpMode) & ~OPMODE_MASK) | mode);
}

static void opmodeLora() {
    u1_t u = OPMODE_LORA;
    u |= 0x8;   // TBD: sx1276 high freq
    writeReg(RegOpMode, u);
}

// configure LoRa modem (cfg1, cfg2)
static void configLoraModem () {
        u1_t mc1 = 0, mc2 = 0, mc3 = 0;
				
				mc1 |= sx127x_bw;
				mc1 |= sx127x_coding;
        mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
				writeReg(LORARegPayloadLength, 15);

        // set ModemConfig1
        writeReg(LORARegModemConfig1, mc1);

        mc2 = sx127x_spreadingFactor;
	      mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
        writeReg(LORARegModemConfig2, mc2);
        
//        mc3 = SX1276_MC3_AGCAUTO | SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
        mc3 = SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
        writeReg(LORARegModemConfig3, mc3);
				
				writeReg(LORARegPreambleLsb,sx127x_preambleLen);
}

static void configChannel () {
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u8_t frf = ((u8_t)sx127x_freq << 19) / 32000000;
    writeReg(RegFrfMsb, (u1_t)(frf>>16));
    writeReg(RegFrfMid, (u1_t)(frf>> 8));
    writeReg(RegFrfLsb, (u1_t)(frf>> 0));
}



static void configPower () {
    // no boost used for now
    s1_t pw = (s1_t) sx127x_power;
    if(pw >= 17) {
        pw = 15;
    } else if(pw < 2) {
        pw = 2;
    }
    // check board type for BOOST pin
//    writeReg(RegPaConfig, (u1_t)(0x80|(pw&0xf)));
    writeReg(RegPaConfig, (u1_t)(pw&0xf));
    writeReg(RegPaDac, readReg(RegPaDac)|0x4);
}

static void txlora () {
    // select LoRa modem (from sleep mode)
		//writeReg(RegOpMode, OPMODE_LORA);
    opmodeLora();
//    ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);

    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);
    // configure LoRa modem (cfg1, cfg2)
    configLoraModem();
    // configure frequency
    configChannel();
    // configure output power
    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    configPower();
    // set sync word
    writeReg(LORARegSyncWord, sx127x_syncWord);
    
    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    // mask all IRQs but TxDone
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers    
    writeReg(LORARegFifoTxBaseAddr, 0x00);
    writeReg(LORARegFifoAddrPtr, 0x00);
    
		// TODO: Implement PacketSend
		writeReg(LORARegPayloadLength, sx127x_pktSize);
       
    // download buffer to the radio FIFO
    writeBuf(RegFifo,lora_txbuf, 15);
		
    // enable antenna switch for TX
    hal_pin_rxtx(1);
		
/*		NRF_LOG_PRINTF("reg 0x01: %02x \r\n",readReg(0x01));
		NRF_LOG_PRINTF("reg 0x06: %02x \r\n",readReg(0x06));
		NRF_LOG_PRINTF("reg 0x07: %02x \r\n",readReg(0x07));
		NRF_LOG_PRINTF("reg 0x08: %02x \r\n",readReg(0x08));
		NRF_LOG_PRINTF("reg 0x09: %02x \r\n",readReg(0x09));
		NRF_LOG_PRINTF("reg 0x0A: %02x \r\n",readReg(0x0A));
		NRF_LOG_PRINTF("reg 0x0B: %02x \r\n",readReg(0x0B));
		NRF_LOG_PRINTF("reg 0x0C: %02x \r\n",readReg(0x0C));
		NRF_LOG_PRINTF("reg 0x1C: %02x \r\n",readReg(0x1C));
		NRF_LOG_PRINTF("reg 0x1D: %02x \r\n",readReg(0x1D));
		NRF_LOG_PRINTF("reg 0x1E: %02x \r\n",readReg(0x1E));
		NRF_LOG_PRINTF("reg 0x1F: %02x \r\n",readReg(0x1F));
		NRF_LOG_PRINTF("reg 0x20: %02x \r\n",readReg(0x20));
		NRF_LOG_PRINTF("reg 0x21: %02x \r\n",readReg(0x21));
		NRF_LOG_PRINTF("reg 0x24: %02x \r\n",readReg(0x24));
		NRF_LOG_PRINTF("reg 0x26: %02x \r\n",readReg(0x26));
		
		NRF_LOG_PRINTF("reg 0x4B: %02x \r\n",readReg(0x4B));
		NRF_LOG_PRINTF("reg 0x4D: %02x \r\n",readReg(0x4D));
		NRF_LOG_PRINTF("reg 0x5B: %02x \r\n",readReg(0x5B));*/
		
    
		// now we actually start the transmission
    opmode(OPMODE_TX);
}


enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

static const u1_t rxlorairqmask[] = {
    [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK,
    [RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK,
    [RXMODE_RSSI]   = 0x00,
};

static void rxlora (u1_t rxmode) {
    // select LoRa modem (from sleep mode)
    opmodeLora();
    ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);
    // enter standby mode (warm up))
    opmode(OPMODE_STANDBY);
    // don't use MAC settings at startup
    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
        writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
        writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
    } else { // single or continuous rx mode
        // configure LoRa modem (cfg1, cfg2)
        configLoraModem();
        // configure frequency
        configChannel();
    }
    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN); 
    // set max payload size
    writeReg(LORARegPayloadMaxLength, sx127x_pktSize);
    // use inverted I/Q signal (prevent mote-to-mote communication)

    // XXX: use flag to switch on/off inversion
//    if (LMIC.noRXIQinversion) {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) & ~(1<<6));
//    } else {
//        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ)|(1<<6));
//    }

    // set symbol timeout (for single rx)
		writeReg(LORARegSymbTimeoutLsb, MINRX_SYMS); // TODO: add some guard times
		
    // set sync word
    writeReg(LORARegSyncWord, sx127x_syncWord);
    
    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, ~rxlorairqmask[rxmode]);

    // enable antenna switch for RX
    hal_pin_rxtx(0);

    // now instruct the radio to receive
    if (rxmode == RXMODE_SINGLE) { // single rx
       // hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
				nrf_delay_ms(100); // TODO: why?
				
        opmode(OPMODE_RX_SINGLE);
    } else { // continous rx (scan or rssi)
        opmode(OPMODE_RX); 
    }
		
		 NRF_LOG_RAW_INFO("reg 0x01: %02x \r\n",readReg(0x01));
		 NRF_LOG_RAW_INFO("reg 0x06: %02x \r\n",readReg(0x06));
		 NRF_LOG_RAW_INFO("reg 0x07: %02x \r\n",readReg(0x07));
		 NRF_LOG_RAW_INFO("reg 0x08: %02x \r\n",readReg(0x08));
		 NRF_LOG_RAW_INFO("reg 0x09: %02x \r\n",readReg(0x09));
		 NRF_LOG_RAW_INFO("reg 0x0A: %02x \r\n",readReg(0x0A));
		 NRF_LOG_RAW_INFO("reg 0x0B: %02x \r\n",readReg(0x0B));
		 NRF_LOG_RAW_INFO("reg 0x0C: %02x \r\n",readReg(0x0C));
		 NRF_LOG_RAW_INFO("reg 0x1C: %02x \r\n",readReg(0x1C));
		 NRF_LOG_RAW_INFO("reg 0x1D: %02x \r\n",readReg(0x1D));
		 NRF_LOG_RAW_INFO("reg 0x1E: %02x \r\n",readReg(0x1E));
		 NRF_LOG_RAW_INFO("reg 0x1F: %02x \r\n",readReg(0x1F));
		 NRF_LOG_RAW_INFO("reg 0x20: %02x \r\n",readReg(0x20));
		 NRF_LOG_RAW_INFO("reg 0x21: %02x \r\n",readReg(0x21));
		 NRF_LOG_RAW_INFO("reg 0x24: %02x \r\n",readReg(0x24));
		 NRF_LOG_RAW_INFO("reg 0x26: %02x \r\n",readReg(0x26));
		
		 NRF_LOG_RAW_INFO("reg 0x4B: %02x \r\n",readReg(0x4B));
		 NRF_LOG_RAW_INFO("reg 0x4D: %02x \r\n",readReg(0x4D));
		 NRF_LOG_RAW_INFO("reg 0x5B: %02x \r\n",readReg(0x5B));
		
		/*
reg 0x01: 89  // set to high frequency registers
reg 0x06: d9 
reg 0x07: 61 
reg 0x08: 99 
reg 0x09: 4f // set PA boost (modify PCB)
reg 0x0A: 09 // ramp time correct??
reg 0x0B: 2b // OCP to 240 mA
reg 0x0C: 21 // set LNA boost HF (SET)
reg 0x1C: 00 
reg 0x1D: 73 
reg 0x1E: c4 
reg 0x1F: 05 // timeout OK?
reg 0x20: 00 
reg 0x21: 08 // preamble length OK?
reg 0x24: 00 // freq hop?
reg 0x26: 04 // Auto AGC?
reg 0x4B: 09 // set tcxo!!
reg 0x4D: 84 
reg 0x5B: 00 
		*/
}

// get random seed from wideband noise rssi
void radio_init () {
    hal_disableIRQs();

    // manually reset radio
#ifdef CFG_sx1276_radio
    hal_pin_rst(0); // drive RST pin low
#else
    hal_pin_rst(1); // drive RST pin high
#endif
		nrf_delay_ms(1);
    hal_pin_rst(2); // configure RST pin floating!
		nrf_delay_ms(1);

    opmode(OPMODE_SLEEP);

    // some sanity checks, e.g., read version number
    u1_t v = readReg(RegVersion);
#ifdef CFG_sx1276_radio
    ASSERT(v == 0x12 ); 
#elif CFG_sx1272_radio
    ASSERT(v == 0x22);
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif
		
	  NRF_LOG_RAW_INFO("sx127x_drv: Radio version %02X\r\n",v);
		writeReg(RegTcxo,0x19); // enable TCXO
		writeReg(LORARegPreambleLsb,sx127x_preambleLen);
		
    // seed 15-byte randomness via noise rssi
    rxlora(RXMODE_RSSI);
    while( (readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX ); // continuous rx
    for(int i=1; i<16; i++) {
        for(int j=0; j<8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( (b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01) );
            randbuf[i] = (randbuf[i] << 1) | b;
        }
    }
    randbuf[0] = 16; // set initial index
  
		//#ifdef CFG_sx1276mb1_board
    // chain calibration
   writeReg(RegPaConfig, 0);
    
    // Launch Rx chain calibration for LF band
//    writeReg(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
//    while((readReg(FSKRegImageCal)&RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING){ ; }

    // Sets a Frequency in HF band
    u4_t frf = 868000000;
    writeReg(RegFrfMsb, (u1_t)(frf>>16));
    writeReg(RegFrfMid, (u1_t)(frf>> 8));
    writeReg(RegFrfLsb, (u1_t)(frf>> 0));

    // Launch Rx chain calibration for HF band 
    writeReg(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
    while((readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) { ; }
		//#endif /* CFG_sx1276mb1_board */

    opmode(OPMODE_SLEEP);

    hal_enableIRQs();
}

void sx127x_sendPkt(uint8_t *pktData)
{
	memcpy(lora_txbuf,pktData,sx127x_pktSize);
  writeReg(LORARegIrqFlags, 0xFF); // clear interrupts
	txlora();
}

int8_t sx127x_getPktSNR()
{
	return lora_snr;
}

int8_t sx127x_getPktRSSI()
{
	return lora_rssi;
}

void sx127x_startRx()
{		
	rxlora(RXMODE_SCAN);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//		NRF_LOG_PRINTF(" Interrupt: %02X\r\n",pin);
		
		if (pin == 11) {
	    u1_t flags = readReg(LORARegIrqFlags);
//			NRF_LOG_PRINTF("  flags: %02X\r\n",flags);
			
	    if( flags & IRQ_LORA_TXDONE_MASK ) {
	        // save exact tx time
	        //LMIC.txend = now - us2osticks(43); // TXDONE FIXUP
	    } else if( flags & IRQ_LORA_CRCERR_MASK ) {
//					NRF_LOG_PRINTF("CRC err \r\n");
	    } else if( flags & IRQ_LORA_RXDONE_MASK ) {
	        // save exact rx time
//	        LMIC.rxtime = now;
	        // read the PDU and inform the MAC that we received something
	        lora_dataLen = (readReg(LORARegModemConfig1) & SX1276_MC1_IMPLICIT_HEADER_MODE_ON) ?
	            readReg(LORARegPayloadLength) : readReg(LORARegRxNbBytes);
	        // set FIFO read address pointer
	        writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr)); 
	        // now read the FIFO
	        readBuf(RegFifo, lora_rxbuf, lora_dataLen);
	        // read rx quality parameters
	        lora_snr  = readReg(LORARegPktSnrValue); // SNR [dB] * 4
					lora_snr = lora_snr/4;
	        lora_rssi = readReg(LORARegPktRssiValue) - 157; // RSSI [dBm] (-196...+63)
					if (lora_dataLen==sx127x_pktSize) {
						if (sx127xEventHandler!=NULL) {
							sx127xEventHandler(lora_rxbuf);
						}
					}
					
	    } else if( flags & IRQ_LORA_RXTOUT_MASK ) {
	        // indicate timeout
	        lora_dataLen = 0;
	    }			
		}
    // clear radio IRQ flags		
    writeReg(LORARegIrqFlags, 0xFF); // clear interrupts
}

void sx127x_init(uint32_t pin_mosi, uint32_t pin_miso, uint32_t pin_sck, uint32_t pin_cs, uint32_t rst_pin,void (*eventHandler)(uint8_t *rxPkt))
{
	uint32_t err_code;
	
	nrf_drv_spi_config_t config = NRF_DRV_SPI_DEFAULT_CONFIG;
	config.frequency = NRF_DRV_SPI_FREQ_1M;
	config.mode      = NRF_DRV_SPI_MODE_0;
	config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
	config.mosi_pin = pin_mosi;
	config.miso_pin = pin_miso;
	config.sck_pin = pin_sck;
	config.ss_pin = pin_cs;	
	config.orc = 0xFF; // overrun character

	lora_rst_pin = rst_pin;

	err_code = nrf_drv_spi_init(&m_spi_master_0, &config, 0, NULL); // without event handler: blocking mode!
	if (err_code != NRF_SUCCESS)
	{
			 NRF_LOG_RAW_INFO("SPI FALIED\r\n");
	    // Initialization failed. Take recovery action.
	}
	
	sx127xEventHandler = eventHandler;
	
	radio_init();
		
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
	
  err_code = nrf_drv_gpiote_in_init(11, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);
//  err_code = nrf_drv_gpiote_in_init(12, &in_config, in_pin_handler);
//  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(11, true);
//  nrf_drv_gpiote_in_event_enable(12, true);
	
}
