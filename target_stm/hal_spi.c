
#include "hal.h"
#include "hal_spi.h"
#include "device.h"

#if defined(CFG_sx126x_radio)
#include "radio_sx126x.h"

#define HAL_PIN_BUSY    HAL_GPIO_ReadPin(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == GPIO_PIN_SET

#endif

static SPI_HandleTypeDef SpiHandle;

/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
GPIO_InitTypeDef  GPIO_InitStruct;

  if(hspi->Instance == SPIx)
  {     
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI clock */
    SPIx_CLK_ENABLE(); 
    
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
  * @brief SPI MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPIx)
  {   
    /*##-1- Reset peripherals ##################################################*/
    SPIx_FORCE_RESET();
    SPIx_RELEASE_RESET();
    
    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
  }
}

void hal_spi_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    NSS_PORT_ENABLE;

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = NSS_PIN;
    HAL_GPIO_Init(NSS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(NSS_PORT, NSS_PIN, GPIO_PIN_SET);

    /*##-1- Configure the SPI peripheral #######################################*/
    /* Set the SPI parameters */
    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;

    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
        /* Initialization Error */
        for (;;) asm("nop");
    }
}

#define NSS_ASSERT       HAL_GPIO_WritePin(NSS_PORT, NSS_PIN, GPIO_PIN_RESET)
#define NSS_RELEASE      HAL_GPIO_WritePin(NSS_PORT, NSS_PIN, GPIO_PIN_SET)

static inline u1_t hal_spi (u1_t out)
{
    SpiHandle.Instance->DR = out;

    while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_RXNE) == RESET)
        ;

    return SpiHandle.Instance->DR;
}

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void writeReg (u1_t addr, u1_t data )
{
    __HAL_SPI_ENABLE(&SpiHandle);
    NSS_ASSERT;
    hal_spi(addr | 0x80);
    hal_spi(data);
    NSS_RELEASE;
    __HAL_SPI_DISABLE(&SpiHandle);
}

u1_t readReg (u1_t addr)
{
    __HAL_SPI_ENABLE(&SpiHandle);
    NSS_ASSERT;
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    NSS_RELEASE;
    __HAL_SPI_DISABLE(&SpiHandle);
    return val;
}

void writeBuf (u1_t addr, xref2u1_t buf, u1_t len)
{
    __HAL_SPI_ENABLE(&SpiHandle);
    NSS_ASSERT;
    hal_spi(addr | 0x80);
    for (u1_t i=0; i<len; i++) {
        hal_spi(buf[i]);
    }
    NSS_RELEASE;
    __HAL_SPI_DISABLE(&SpiHandle);
}

void readBuf (u1_t addr, xref2u1_t buf, u1_t len)
{
    __HAL_SPI_ENABLE(&SpiHandle);
    NSS_ASSERT;
    hal_spi(addr & 0x7F);
    for (u1_t i=0; i<len; i++) {
        buf[i] = hal_spi(0x00);
    }
    NSS_RELEASE;
    __HAL_SPI_DISABLE(&SpiHandle);
}

#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

#if defined(CFG_sx126x_radio)
void readBuf(uint8_t size, uint8_t offset, uint8_t* out)
{
    unsigned i;

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

    __HAL_SPI_DISABLE(&SpiHandle);
}

void writeBuf(uint8_t size, uint8_t offset, uint8_t* in)
{
    uint8_t i;

    __HAL_SPI_ENABLE(&SpiHandle);

    while (HAL_PIN_BUSY)
        ;

    NSS_ASSERT;
    hal_spi(OPCODE_WRITE_BUFFER);
    hal_spi(offset);
    for (i = 0; i < size; i++) {
        hal_spi(in[i]);
    }

    NSS_RELEASE;

    __HAL_SPI_DISABLE(&SpiHandle);
}

void radio_xfer(uint8_t opcode, uint8_t wlen, uint8_t rlen, uint8_t* ptr)
{
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
}
#endif /* CFG_sx126x_radio */

