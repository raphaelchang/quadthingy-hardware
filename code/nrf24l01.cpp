#include "nrf24l01.h"

#include "ch.h"
#include "hal.h"

static const SPIConfig nrf24l01SPI =
{
    NULL,
    GPIOA,
    GPIOA_LRCK,
    SPI_CR1_BR_0
};

static const NRF24L01Config nrf24l01Config =
{
    &SPID1,
    GPIOB,
    GPIOB_PIN1
};

NRF24L01Driver nrf24l01;
static const uint8_t addr[5] = "QUAD";

static void nrfExtCallback(EXTDriver *extp, expchannel_t channel)
{
    nrf24l01ExtIRQ(&nrf24l01);
}

static const EXTConfig extcfg =
{
    {
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, nrfExtCallback},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL}
    }
};

void initNRF24L01(void)
{
    palSetPadMode(GPIOA, GPIOA_LRCK, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOA, GPIOA_LRCK);
    palSetPadMode(GPIOB, GPIOB_PIN1, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(GPIOB, GPIOB_PIN1);
    palSetPadMode(GPIOC, GPIOC_PIN5, PAL_MODE_INPUT_PULLUP);

    spiStart(&SPID1, &nrf24l01SPI);
    extStart(&EXTD1, &extcfg);
    nrf24l01ObjectInit(&nrf24l01);
    nrf24l01Start(&nrf24l01, &nrf24l01Config);
    extChannelEnable(&EXTD1, 5);

    nrf24l01EnableDynamicSize(&nrf24l01);
    nrf24l01EnableDynamicPipeSize(&nrf24l01, 0x3f);

    nrf24l01SetTXAddress(&nrf24l01, addr);
    nrf24l01SetRXAddress(&nrf24l01, 0, addr);
    nrf24l01SetPayloadSize(&nrf24l01, 0, 32);

    nrf24l01FlushRX(&nrf24l01);
    nrf24l01FlushTX(&nrf24l01);
    nrf24l01ClearIRQ(&nrf24l01, NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT);

    nrf24l01PowerUp(&nrf24l01);
}

static size_t writet(void *instance, const uint8_t *bp, size_t n, systime_t time)
{
    event_listener_t nrfListener;
    NRF24L01Driver *nrfp = ((struct NRF24L01Channel*)instance)->nrfp;
    if (instance != &nrfp->channels[0])
        return 0;

    chEvtRegisterMask(&nrfp->eventSource, &nrfListener, NRF24L01_EVENT_IRQ);

    uint16_t sended = 0;
    uint8_t status_reg;

    status_reg = nrf24l01GetStatus(nrfp);

    if (status_reg & NRF24L01_TX_FULL)
    {
        chEvtUnregister(&nrfp->eventSource, &nrfListener);
        return 0;
    }

    nrf24l01EnterTX(nrfp);

    while (sended < n)
    {
        uint8_t psize = (n - sended > 32) ? 32 : n - sended;

        nrf24l01WritePayload(nrfp, psize, bp + sended);

        if (!chEvtWaitOneTimeout(NRF24L01_EVENT_IRQ, time))
        {
            chEvtUnregister(&nrfp->eventSource, &nrfListener);
            nrf24l01EnterStandby(nrfp);
            return sended;
        }

        status_reg = nrf24l01GetStatus(nrfp);

        if (status_reg & NRF24L01_MAX_RT)
        {
            chEvtUnregister(&nrfp->eventSource, &nrfListener);
            nrf24l01ClearIRQ(nrfp, NRF24L01_MAX_RT);
            nrf24l01FlushTX(nrfp);
            nrf24l01EnterStandby(nrfp);
            return sended;
        }

        if (status_reg & NRF24L01_TX_DS)
            nrf24l01ClearIRQ(nrfp, NRF24L01_TX_DS);

        sended += psize;
    }

    nrf24l01EnterStandby(nrfp);

    chEvtBroadcastFlags(&((struct NRF24L01Channel*)instance)->event, CHN_OUTPUT_EMPTY | CHN_TRANSMISSION_END);
    chEvtUnregister(&nrfp->eventSource, &nrfListener);

    return sended;
}

static size_t readt(void *instance, uint8_t *bp, size_t n, systime_t time)
{
    event_listener_t nrfListener;
    NRF24L01Driver *nrfp = ((struct NRF24L01Channel*)instance)->nrfp;
    struct NRF24L01Channel *chp = NULL;

    for (int i = 0; i < 6; i++)
    {
        if (&nrfp->channels[i] == instance)
        {
            chp = &nrfp->channels[i];
            break;
        }
    }

    if (!chp)
        return 0;

    uint16_t received = 0;

    chEvtRegisterMask(&nrfp->eventSource, &nrfListener, NRF24L01_EVENT_IRQ);

    nrf24l01EnterRX(nrfp);

    while (received < n)
    {
        if (chp->rxBufCount)
        {
            uint8_t len = (n - received > chp->rxBufCount) ? chp->rxBufCount : n - received;
            for (uint8_t i = 0; i < len; i++)
                bp[received + i] = chp->rxBuf[(32 - chp->rxBufCount) + i];
            chp->rxBufCount -= len;
            received += len;
            continue;
        }
        uint8_t status_reg = nrf24l01GetFIFOStatus(nrfp);

        if (!(status_reg & NRF24L01_FIFO_RX_EMPTY))
        {
            uint8_t pipe = (status_reg >> 1) & 0x7;
            uint8_t packetSize = nrf24l01GetSize(nrfp);
            struct NRF24L01Channel *rxcp;
            rxcp = &nrfp->channels[pipe];

            if (rxcp != instance)
            {
                if ((rxcp->rxBufCount + packetSize > 32) || !rxcp->rxBufCount)
                {
                    //Overrun error OR empty buffer
                    nrf24l01ReadPayload(nrfp, packetSize, rxcp->rxBuf + 32 - packetSize, &pipe);
                    rxcp->rxBufCount = packetSize;
                }
                else
                {
                    //Move buffer and append to end
                    for (uint8_t i = (32 - (rxcp->rxBufCount + packetSize)); i < 32 - packetSize; i++)
                        rxcp->rxBuf[i] = rxcp->rxBuf[i + packetSize];

                    nrf24l01ReadPayload(nrfp, packetSize, rxcp->rxBuf + 32 - packetSize, &pipe);
                    rxcp->rxBufCount += packetSize;
                }
                chEvtBroadcastFlags(&rxcp->event, CHN_INPUT_AVAILABLE);
            }
            else
            {
                if (packetSize <= n - received)
                {
                    nrf24l01ReadPayload(nrfp, packetSize, bp + received, &pipe);
                    received += packetSize;
                }
                else
                {
                    nrf24l01ReadPayload(nrfp, packetSize, rxcp->rxBuf + (32 - packetSize), &pipe);
                    rxcp->rxBufCount = packetSize;
                }
            }
            continue;
        }

        if (!chEvtWaitOneTimeout(NRF24L01_EVENT_IRQ, time))
        {
            chEvtUnregister(&nrfp->eventSource, &nrfListener);
            nrf24l01EnterStandby(nrfp);
            return received;
        }

        status_reg = nrf24l01GetStatus(nrfp);

        if (status_reg & NRF24L01_RX_DR)
            nrf24l01ClearIRQ(nrfp, NRF24L01_RX_DR);
    }

    if (chp->rxBufCount)
        chEvtBroadcastFlags(&chp->event, CHN_INPUT_AVAILABLE);

    nrf24l01EnterStandby(nrfp);
    chEvtUnregister(&nrfp->eventSource, &nrfListener);

    return received;
}

static msg_t putt(void *instance, uint8_t b, systime_t time)
{
    if (!writet(instance, &b, 1, time))
        return Q_TIMEOUT;
    return Q_OK;
}

static msg_t gett(void *instance, systime_t time)
{
    NRF24L01Driver *nrfp = ((struct NRF24L01Channel*)instance)->nrfp;
    struct NRF24L01Channel *chp = NULL;

    for (int i = 0; i < 6; i++)
    {
        if (&nrfp->channels[i] == instance)
        {
            chp = &nrfp->channels[i];
            break;
        }
    }

    if (!chp)
        return Q_RESET;

    if (chp->rxBufCount)
    {
        uint8_t b = chp->rxBuf[32 - chp->rxBufCount];
        chp->rxBufCount--;
        return b;
    }
    else
    {
        uint8_t b;
        if (!readt(instance, &b, 1, time))
            return Q_TIMEOUT;
        return b;
    }
}

static size_t write(void *instance, const uint8_t *bp, size_t n)
{
    return writet(instance, bp, n, TIME_INFINITE);
}

static size_t read(void *instance, uint8_t *bp, size_t n)
{
    return readt(instance, bp, n, TIME_INFINITE);
}

static msg_t put(void *instance, uint8_t b)
{
    return putt(instance, b, TIME_INFINITE);
}

static msg_t get(void *instance)
{
    return gett(instance, TIME_INFINITE);
}

static const struct NRF24L01ChannelVMT vmt = {write, read, put, get,
    putt, gett, writet, readt
};

void nrf24l01ObjectInit(NRF24L01Driver *nrfp)
{
    for (int i = 0; i < 6; i++)
    {
        nrfp->channels[i].vmt = &vmt;
        nrfp->channels[i].rxBufCount = 0;
        nrfp->channels[i].nrfp = nrfp;
        chEvtObjectInit(&nrfp->channels[i].event);
    }

    chEvtObjectInit(&nrfp->eventSource);
}

void nrf24l01Start(NRF24L01Driver *nrfp, const NRF24L01Config *config)
{
    nrfp->config = config;
}

void nrf24l01WriteRegister(NRF24L01Driver *nrfp, uint8_t reg, uint8_t value)
{
    uint8_t op[2] = {NRF24L01_CMD_W_REGISTER | (reg & 0x1F), value};

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 2, op);
    spiUnselect(nrfp->config->spip);
}

uint8_t nrf24l01ReadRegister(NRF24L01Driver *nrfp, uint8_t reg)
{
    uint8_t op[2] = {NRF24L01_CMD_R_REGISTER | (reg & 0x1F), 0};
    uint8_t data[2] = {0, 0};

    spiSelect(nrfp->config->spip);
    spiExchange(nrfp->config->spip, 2, op, data);
    spiUnselect(nrfp->config->spip);

    return data[1];
}

void nrf24l01WriteAddressRegister(NRF24L01Driver *nrfp, uint8_t reg, const uint8_t value[4])
{
    uint8_t op = NRF24L01_CMD_W_REGISTER | (reg & 0x1F);

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 1, &op);
    spiSend(nrfp->config->spip, 5, value);
    spiUnselect(nrfp->config->spip);
}

void nrf24l01ReadAddressRegister(NRF24L01Driver *nrfp, uint8_t reg, uint8_t value[4])
{
    uint8_t op = NRF24L01_CMD_R_REGISTER | (reg & 0x1F);

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 1, &op);
    spiReceive(nrfp->config->spip, 5, value);
    spiUnselect(nrfp->config->spip);
}

uint8_t nrf24l01GetStatus(NRF24L01Driver *nrfp)
{
    uint8_t op = NRF24L01_CMD_NOP;
    uint8_t data;

    spiSelect(nrfp->config->spip);
    spiExchange(nrfp->config->spip, 1, &op, &data);
    spiUnselect(nrfp->config->spip);

    return data;
}

void nrf24l01PowerUp(NRF24L01Driver *nrfp)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_CONFIG);
    reg |= NRF24L01_PWR_UP;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_CONFIG, reg);
}

void nrf24l01PowerDown(NRF24L01Driver *nrfp)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_CONFIG);
    reg &= ~(NRF24L01_PWR_UP);
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_CONFIG, reg);
}

void nrf24l01SetupRetransmit(NRF24L01Driver *nrfp, uint8_t delay, uint8_t count)
{
    uint8_t reg = ((delay & 0xF) << 4) | (count & 0xF);
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_SETUP_RETR, reg);
}

void nrf24l01SetChannel(NRF24L01Driver *nrfp, uint8_t channel)
{
    uint8_t reg = channel & 0x7F;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_RF_CH, reg);
}

void nrf24l01ClearIRQ(NRF24L01Driver *nrfp, uint8_t irq)
{
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_STATUS, irq & (NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT));
}

void nrf24l01SetRXAddress(NRF24L01Driver *nrfp, uint8_t pipe, const uint8_t addr[5])
{
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_SETUP_AW, 0b10000000);
    if ((pipe == 0) || (pipe == 1))
        nrf24l01WriteAddressRegister(nrfp, NRF24L01_REG_RX_ADDR_P0 + pipe, addr);
    else if ((pipe > 2) && (pipe < 6))
        nrf24l01WriteRegister(nrfp, NRF24L01_REG_RX_ADDR_P0 + pipe, *addr);
}

void nrf24l01SetTXAddress(NRF24L01Driver *nrfp, const uint8_t addr[5])
{
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_SETUP_AW, 0b10000000);
    return nrf24l01WriteAddressRegister(nrfp, NRF24L01_REG_TX_ADDR, addr);
}

void nrf24l01SetPayloadSize(NRF24L01Driver *nrfp, uint8_t pipe, uint8_t size)
{
    if ((pipe > 5) || (size > 32))
        return;

    return nrf24l01WriteRegister(nrfp, NRF24L01_REG_RX_PW_P0 + pipe, size);
}

uint8_t nrf24l01GetFIFOStatus(NRF24L01Driver *nrfp)
{
    return nrf24l01ReadRegister(nrfp, NRF24L01_REG_FIFO_STATUS);
}

void nrf24l01EnableDynamicSize(NRF24L01Driver *nrfp)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_FEATURE);

    if (reg & NRF24L01_EN_DPL)
        return;

    reg |= NRF24L01_EN_DPL;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_FEATURE, reg);

    reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_FEATURE);
    if ((reg & NRF24L01_EN_DPL) == 0)
    {
        nrf24l01ToggleFeatures(nrfp);
        nrf24l01WriteRegister(nrfp, NRF24L01_REG_FEATURE, reg);
    }
}

void nrf24l01EnableDynamicPipeSize(NRF24L01Driver *nrfp, uint8_t pipes)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_DYNPD);
    reg |= pipes & 0x3F;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_DYNPD, reg);
}

void nrf24l01DisableDynamicSize(NRF24L01Driver *nrfp)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_FEATURE);
    reg &= ~NRF24L01_EN_DPL;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_FEATURE, reg);
}

void nrf24l01DisableDynamicPipeSize(NRF24L01Driver *nrfp, uint8_t pipes)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_DYNPD);
    reg &= ~(pipes & 0x3F);
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_DYNPD, reg);
}

void nrf24l01ReadPayload(NRF24L01Driver *nrfp, uint8_t size, uint8_t *data, uint8_t *pipe)
{
    if (size > 32)
        return;

    uint8_t op = NRF24L01_CMD_R_RX_PAYLOAD;
    uint8_t status;

    spiSelect(nrfp->config->spip);
    spiExchange(nrfp->config->spip, 1, &op, &status);
    spiReceive(nrfp->config->spip, size, data);
    spiUnselect(nrfp->config->spip);

    *pipe = (status >> 1) & 0x7;
}

void nrf24l01WritePayload(NRF24L01Driver *nrfp, uint8_t size, const uint8_t *data)
{
    if (size > 32)
        return;

    uint8_t op = NRF24L01_CMD_W_TX_PAYLOAD;

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 1, &op);
    spiSend(nrfp->config->spip, size, data);
    spiUnselect(nrfp->config->spip);
}

void nrf24l01ToggleFeatures(NRF24L01Driver *nrfp)
{
    uint8_t op[2] = {NRF24L01_CMD_ACTIVATE, 0x73};

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 2, op);
    spiUnselect(nrfp->config->spip);
}

void nrf24l01FlushRX(NRF24L01Driver *nrfp)
{
    uint8_t op = NRF24L01_CMD_FLUSH_RX;

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 1, &op);
    spiUnselect(nrfp->config->spip);
}

void nrf24l01FlushTX(NRF24L01Driver *nrfp)
{
    uint8_t op = NRF24L01_CMD_FLUSH_TX;

    spiSelect(nrfp->config->spip);
    spiSend(nrfp->config->spip, 1, &op);
    spiUnselect(nrfp->config->spip);
}

uint8_t nrf24l01GetSize(NRF24L01Driver *nrfp)
{
    uint8_t op[2] = {NRF24L01_CMD_R_RX_PL_WID, 0};
    uint8_t data[2] = {0, 0};

    spiSelect(nrfp->config->spip);
    spiExchange(nrfp->config->spip, 2, op, data);
    spiUnselect(nrfp->config->spip);

    return data[1];
}

void nrf24l01EnterRX(NRF24L01Driver *nrfp)
{
    palClearPad(nrfp->config->cePort, nrfp->config->cePad);

    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_CONFIG);
    reg |= NRF24L01_PRIM_RX;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_CONFIG, reg);

    palSetPad(nrfp->config->cePort, nrfp->config->cePad);
}

void nrf24l01EnterTX(NRF24L01Driver *nrfp)
{
    palClearPad(nrfp->config->cePort, nrfp->config->cePad);

    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_CONFIG);
    reg &= ~NRF24L01_PRIM_RX;
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_CONFIG, reg);

    palSetPad(nrfp->config->cePort, nrfp->config->cePad);
}

void nrf24l01EnterStandby(NRF24L01Driver *nrfp)
{
    palClearPad(nrfp->config->cePort, nrfp->config->cePad);
}

void nrf24l01EnablePipes(NRF24L01Driver *nrfp, uint8_t pipes)
{
    uint8_t reg = nrf24l01ReadRegister(nrfp, NRF24L01_REG_EN_RXADDR);
    reg &= ~(pipes & 0x3F);
    nrf24l01WriteRegister(nrfp, NRF24L01_REG_EN_RXADDR, reg);
}

void nrf24l01ExtIRQ(NRF24L01Driver *nrfp)
{
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&nrfp->eventSource, NRF24L01_EVENT_IRQ);
    chSysUnlockFromISR();
}