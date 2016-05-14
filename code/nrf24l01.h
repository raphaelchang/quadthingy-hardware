#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"

#define NRF24L01_CMD_R_REGISTER 0x00
#define NRF24L01_CMD_W_REGISTER 0x20
#define NRF24L01_CMD_R_RX_PAYLOAD 0x61
#define NRF24L01_CMD_W_TX_PAYLOAD 0xA0
#define NRF24L01_CMD_FLUSH_TX 0xE1
#define NRF24L01_CMD_FLUSH_RX 0xE2
#define NRF24L01_CMD_REUSE_TX_PL 0xE3
#define NRF24L01_CMD_ACTIVATE 0x50
#define NRF24L01_CMD_R_RX_PL_WID 0x60
#define NRF24L01_CMD_W_ACK_PAYLOAD 0xA8
#define NRF24L01_CMD_W_TX_PAYLOAD_NO_ACK 0xB0
#define NRF24L01_CMD_NOP 0xFF

#define NRF24L01_REG_CONFIG 0x00
#define NRF24L01_REG_EN_AA 0x01
#define NRF24L01_REG_EN_RXADDR 0x02
#define NRF24L01_REG_SETUP_AW 0x03
#define NRF24L01_REG_SETUP_RETR 0x04
#define NRF24L01_REG_RF_CH 0x05
#define NRF24L01_REG_RF_SETUP 0x06
#define NRF24L01_REG_STATUS 0x07
#define NRF24L01_REG_OBSERVE_TX 0x08
#define NRF24L01_REG_CD 0x09
#define NRF24L01_REG_RX_ADDR_P0 0x0A
#define NRF24L01_REG_RX_ADDR_P1 0x0B
#define NRF24L01_REG_RX_ADDR_P2 0x0C
#define NRF24L01_REG_RX_ADDR_P3 0x0D
#define NRF24L01_REG_RX_ADDR_P4 0x0E
#define NRF24L01_REG_RX_ADDR_P5 0x0F
#define NRF24L01_REG_TX_ADDR 0x10
#define NRF24L01_REG_RX_PW_P0 0x11
#define NRF24L01_REG_RX_PW_P1 0x12
#define NRF24L01_REG_RX_PW_P2 0x13
#define NRF24L01_REG_RX_PW_P3 0x14
#define NRF24L01_REG_RX_PW_P4 0x15
#define NRF24L01_REG_RX_PW_P5 0x16
#define NRF24L01_REG_FIFO_STATUS 0x17
#define NRF24L01_REG_DYNPD 0x1C
#define NRF24L01_REG_FEATURE 0x1D

// CONFIG
#define NRF24L01_MASK_RX_DR 0x40
#define NRF24L01_MASK_TX_DS 0x20
#define NRF24L01_MASK_MAX_RT 0x10
#define NRF24L01_EN_CRC 0x08
#define NRF24L01_CRCO 0x04
#define NRF24L01_PWR_UP 0x02
#define NRF24L01_PRIM_RX 0x01

// EN_AA
#define NRF24L01_ENAA_P5 0x20
#define NRF24L01_ENAA_P4 0x10
#define NRF24L01_ENAA_P3 0x08
#define NRF24L01_ENAA_P2 0x04
#define NRF24L01_ENAA_P1 0x02
#define NRF24L01_ENAA_P0 0x01

// EN_RXADDR
#define NRF24L01_ERX_P5 0x20
#define NRF24L01_ERX_P4 0x10
#define NRF24L01_ERX_P3 0x08
#define NRF24L01_ERX_P2 0x04
#define NRF24L01_ERX_P1 0x02
#define NRF24L01_ERX_P0 0x01

// RF_SETUP
#define NRF24L01_PLL_LOCK 0x10
#define NRF24L01_RF_DR 0x8
#define NRF24L01_RF_PWR 0x6
#define NRF24L01_LNA_HCURR 0x01

// STATUS
#define NRF24L01_RX_DR 0x40
#define NRF24L01_TX_DS 0x20
#define NRF24L01_MAX_RT 0x10
#define NRF24L01_RX_P_NO 0x0E
#define NRF24L01_TX_FULL 0x01

// FIFO_STATUS
#define NRF24L01_FIFO_TX_REUSE 0x40
#define NRF24L01_FIFO_TX_FULL 0x20
#define NRF24L01_FIFO_TX_EMPTY 0x10
#define NRF24L01_FIFO_RX_FULL 0x20
#define NRF24L01_FIFO_RX_EMPTY 0x01

// FEATURE
#define NRF24L01_EN_DPL 0x4
#define NRF24L01_EN_ACK_PAY 0x2
#define NRF24L01_EN_DYN_ACK 0x1

#define NRF24L01_EVENT_IRQ 0x01

typedef struct
{
    SPIDriver             *spip;
    ioportid_t            cePort;
    uint16_t              cePad;
} NRF24L01Config;

struct NRF24L01ChannelVMT
{
    _base_asynchronous_channel_methods
};

struct NRF24L01Channel
{
    const struct NRF24L01ChannelVMT *vmt;
    struct NRF24L01Driver *nrfp;
    uint8_t rxBuf[32];
    uint8_t rxBufCount;
    _base_asynchronous_channel_data
};

struct NRF24L01Driver
{
    struct NRF24L01Channel channels[6];
    event_source_t eventSource;
    const NRF24L01Config *config;
};

typedef struct NRF24L01Driver NRF24L01Driver;

extern NRF24L01Driver nrf24l01;

void initNRF24L01(void);
void nrf24l01ExtIRQ(NRF24L01Driver *nrfp);

void nrf24l01ObjectInit(NRF24L01Driver *nrfp);
void nrf24l01Start(NRF24L01Driver *nrfp, const NRF24L01Config *config);

void nrf24l01WriteRegister(NRF24L01Driver *nrfp, uint8_t reg, uint8_t value);
uint8_t nrf24l01ReadRegister(NRF24L01Driver *nrfp, uint8_t reg);
void nrf24l01WriteAddressRegister(NRF24L01Driver *nrfp, uint8_t reg, const uint8_t value[5]);
void nrf24l01ReadAddressRegister(NRF24L01Driver *nrfp, uint8_t reg, uint8_t value[5]);

uint8_t nrf24l01GetStatus(NRF24L01Driver *nrfp);
uint8_t nrf24l01GetFIFOStatus(NRF24L01Driver *nrfp);
uint8_t nrf24l01GetSize(NRF24L01Driver *nrfp);

void nrf24l01SetupRetransmit(NRF24L01Driver *nrfp, uint8_t delay, uint8_t count);
void nrf24l01SetChannel(NRF24L01Driver *nrfp, uint8_t channel);

void nrf24l01ClearIRQ(NRF24L01Driver *nrfp, uint8_t irq);

void nrf24l01SetRXAddress(NRF24L01Driver *nrfp, uint8_t pipe, const uint8_t addr[5]);
void nrf24l01SetTXAddress(NRF24L01Driver *nrfp,  const uint8_t addr[5]);
void nrf24l01SetPayloadSize(NRF24L01Driver *nrfp, uint8_t pipe, uint8_t size);
void nrf24l01EnablePipes(NRF24L01Driver *nrfp, uint8_t pipes);

void nrf24l01EnableDynamicSize(NRF24L01Driver *nrfp);
void nrf24l01EnableDynamicPipeSize(NRF24L01Driver *nrfp, uint8_t pipes);
void nrf24l01DisableDynamicSize(NRF24L01Driver *nrfp);
void nrf24l01DisableDynamicPipeSize(NRF24L01Driver *nrfp, uint8_t pipes);

void nrf24l01ReadPayload(NRF24L01Driver *nrfp, uint8_t size, uint8_t *data, uint8_t *pipe);
void nrf24l01WritePayload(NRF24L01Driver *nrfp, uint8_t size, const uint8_t *data);

void nrf24l01ToggleFeatures(NRF24L01Driver *nrfp);

void nrf24l01FlushRX(NRF24L01Driver *nrfp);
void nrf24l01FlushTX(NRF24L01Driver *nrfp);

void nrf24l01PowerUp(NRF24L01Driver *nrfp);
void nrf24l01PowerDown(NRF24L01Driver *nrfp);
void nrf24l01EnterRX(NRF24L01Driver *nrfp);
void nrf24l01EnterTX(NRF24L01Driver *nrfp);
void nrf24l01EnterStandby(NRF24L01Driver *nrfp);

void nrf24l01SetAutoAck(NRF24L01Driver *nrfp, bool enable);
void nrf24l01SetCRC(NRF24L01Driver *nrfp, bool enable);

#endif