
#include "hal.h"
#include "hal_spi.h"
#include "device.h"

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

#if defined(CFG_sx126x_radio)
#include "radio_sx126x.h"

#define HAL_PIN_BUSY        (NRF_GPIO->IN >> RADIO_BUSY_PIN) & 0x1UL

#endif

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void writeReg (u1_t addr, u1_t data )
{
    uint8_t buf[2];

    buf[0] = addr | 0x80;
    buf[1] = data;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buf, 2, NULL, 0));
}

u1_t readReg (u1_t addr)
{
    uint8_t outbuf[2];
    uint8_t inbuf[2];
    outbuf[0] = addr & 0x7f;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, outbuf, 2, inbuf, 2));
    return inbuf[1];
}

void writeBuf (u1_t addr, xref2u1_t buf, u1_t len)
{
    uint8_t outbuf[257];
    outbuf[0] = addr | 0x80;
    memcpy(outbuf+1, buf, len);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, outbuf, len+1, NULL, 0));
}

void readBuf (u1_t addr, xref2u1_t buf, u1_t len)
{
    uint8_t adr = addr & 0x7f;
    uint8_t inbuf[257];
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &adr, 1, inbuf, len+1));
    memcpy(buf, inbuf+1, len);
}
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

#if defined(CFG_sx126x_radio)
void readBuf(uint8_t size, uint8_t offset, uint8_t* out)
{
/*    unsigned i;

    __HAL_SPI_ENABLE(&SpiHandle);

    while (HAL_PIN_BUSY)
        ;

    NSS_ASSERT;

    hal_spi(OPCODE_READ_BUFFER);
    hal_spi(offset);
    hal_spi(0);   // NOP
    i = 0;
    for (i = 0; i < size; i++) {
        out[i] = hal_spi(0);
    }

    NSS_RELEASE;

    __HAL_SPI_DISABLE(&SpiHandle);*/
    uint8_t outbuf[3];
    uint8_t inbuf[259];

    outbuf[0] = OPCODE_READ_BUFFER;
    outbuf[1] = offset;
    outbuf[2] = 0;  // NOP

    while (HAL_PIN_BUSY)
        ;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, outbuf, 3, inbuf, size+3));

    memcpy(out, inbuf+3, size);
}

void writeBuf(uint8_t size, uint8_t offset, uint8_t* in)
{
    uint8_t outbuf[258];
/*    uint8_t i;

    __HAL_SPI_ENABLE(&SpiHandle);

    while (HAL_PIN_BUSY)
        ;

    NSS_ASSERT;
    hal_spi(OPCODE_WRITE_BUFFER);
    hal_spi(0);   // offset
    for (i = 0; i < size; i++) {
        hal_spi(in[i]);
    }

    NSS_RELEASE;

    __HAL_SPI_DISABLE(&SpiHandle);*/
    outbuf[0] = OPCODE_WRITE_BUFFER;
    outbuf[1] = offset;
    memcpy(outbuf+2, in, size);

    while (HAL_PIN_BUSY)
        ;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, outbuf, size+2, NULL, 0));
}

void radio_xfer(uint8_t opcode, uint8_t wlen, uint8_t rlen, uint8_t* ptr)
{
    static bool sleeping = false;
#if 0
    const uint8_t* stopPtr;
    const uint8_t* wstop;
    const uint8_t* rstop;
    uint8_t nop = 0;
    static bool sleeping;

    __HAL_SPI_ENABLE(&SpiHandle);

    if (sleeping) {
        NSS_ASSERT;
        while (HAL_PIN_BUSY)
            ;
        sleeping = false;
    } else {
        if (opcode != OPCODE_GET_STATUS) {
            while (HAL_PIN_BUSY)
                ;
        }
        NSS_ASSERT;
    }

    hal_spi(opcode);

    wstop = ptr + wlen;
    rstop = ptr + rlen;
    if (rlen > wlen)
        stopPtr = rstop;
    else
        stopPtr = wstop;

    for (; ptr < stopPtr; ptr++) {
        if (ptr < wstop && ptr < rstop)
            *ptr = hal_spi(*ptr);
        else if (ptr < wstop)
            hal_spi(*ptr);
        else
            *ptr = hal_spi(nop);    // n >= write length: send NOP
    }

    NSS_RELEASE;

    if (opcode == OPCODE_SET_SLEEP)
        sleeping = true;

    __HAL_SPI_DISABLE(&SpiHandle);
#endif /* if 0 */
    uint8_t outbuf[257];
    uint8_t inbuf[257];

    outbuf[0] = opcode;
    memcpy(outbuf+1, ptr, wlen);
    if (rlen > wlen)
        memset(outbuf+1+wlen, 0, rlen-wlen);

    if (sleeping) {
        //printf("sleeping\r\n");

        NRF_GPIO->OUTCLR = (1 << SPI_SS_PIN);

        while (HAL_PIN_BUSY)
            ;
        sleeping = false;
    } else {
        if (opcode != OPCODE_GET_STATUS) {
            while (HAL_PIN_BUSY)
                ;
        }
    }

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, outbuf, wlen+1, inbuf, rlen+1));

    memcpy(ptr, inbuf+1, rlen);

    //printf("op%02x ", opcode);
    if (opcode == OPCODE_SET_SLEEP) {
        //printf("opcodeSleep");
        //asm("nop");
        sleeping = true;
    }
    //printf("\r\n");
}
#endif /* CFG_sx126x_radio */

void hal_spi_init()
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.miso_pin = SPI_MISO_PIN;  // D12 p24
    spi_config.mosi_pin = SPI_MOSI_PIN;  // D11 p23
    spi_config.sck_pin  = SPI_SCK_PIN; // D13 p25
    spi_config.ss_pin  = SPI_SS_PIN; // nRF SDK doesnt implement software nSS
    spi_config.frequency = NRF_SPI_FREQ_1M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
}
