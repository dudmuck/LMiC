
#include "debug.h"
#include "device.h"
#include "lmic.h"

UART_HandleTypeDef UartHandle;


#define UART_TIMEOUT_VALUE  1000 /* 1 Second */

/**
  * @brief  Retargets the C library printf function to the USARTx.
  * @param  ch: character to send
  * @param  f: pointer to file (not used)
  * @retval The character transmitted
  */
#ifdef __GNUC__

/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
    HAL_StatusTypeDef ret;
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    ret = HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, UART_TIMEOUT_VALUE);
    if (ret == HAL_BUSY)
        return -1;
    if (ret != HAL_OK) {
        for (;;) asm("nop");
    }

    return ch;
}

void debug_char(char c) {
#ifdef __GNUC__
    __io_putchar(c);
#else
    fputc(c, stdout);
#endif
}

#ifndef USARTx_IRQHandler
    #error USARTx_IRQHandler 
#endif
void USARTx_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandle);
}

uint8_t aRxBuffer;
volatile uint8_t pcbuf_idx;
volatile int8_t _pcbuf_len;
static volatile int8_t prev_len;
uint8_t pcbuf[64];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (aRxBuffer == 8) {
        if (pcbuf_idx > 0) {
            debug_char(8);
            debug_char(' ');
            debug_char(8);
            pcbuf_idx--;
        }
    } else if (aRxBuffer == 3) {
        _pcbuf_len = -1;
    } else if (aRxBuffer == '\r') {
        if (pcbuf_idx == 0)
            _pcbuf_len = prev_len;
        else {
            pcbuf[pcbuf_idx] = 0;   // null terminate
            prev_len = pcbuf_idx;
            pcbuf_idx = 0;
            _pcbuf_len = prev_len;
        }
    } else if (pcbuf_idx < sizeof(pcbuf)) {
        pcbuf[pcbuf_idx++] = aRxBuffer;
        debug_char(aRxBuffer);
    }

    if(HAL_UART_Receive_IT(UartHandle, &aRxBuffer, 1) != HAL_OK)
    {
        for (;;) asm("nop");
    }
}

void debug_init()
{
    UartHandle.Instance        = USARTx;

    UartHandle.Init.BaudRate   = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    { 
        for (;;) asm("nop");
    }
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    { 
        for (;;) asm("nop");
    }

    debug_str("\r\n============== DEBUG STARTED ==============\r\n");

    if(HAL_UART_Receive_IT(&UartHandle, &aRxBuffer, 1) != HAL_OK)
    {
        for (;;) asm("nop");
    }
}

void debug_str (const char* str) {
    while(*str) {
        debug_char(*str++);
    }
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    /*##-1- Reset peripherals ##################################################*/
    USARTx_FORCE_RESET();
    USARTx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure USART1 Tx as alternate function  */
    HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
    /* Configure USART1 Rx as alternate function  */
    HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

    /*##-3- Disable the NVIC for UART ##########################################*/
    HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();


    /* Enable USARTx clock */
    USARTx_CLK_ENABLE(); 

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/
    /* NVIC for USART */
    HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

int serial_is_tx_ongoing(void)
{
    return 0;
}

void debug_event (int ev)
{
    static const char* const evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
#ifdef JOINEUI
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
#endif /* JOINEUI */
        [EV_RFU1]           = "RFU1",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
        [EV_SCAN_FOUND]     = "SCAN_FOUND",
        [EV_TXSTART]        = "EV_TXSTART",
        [EV_LINKCHECK_ANS]  = "EV_LINKCHECK_ANS ",
        [EV_DEVICE_TIME_ANS]= "EV_DEVICE_TIME_ANS",
    };
    debug_str((ev < sizeof(evnames)/sizeof(evnames[0])) ? evnames[ev] : "EV_UNKNOWN" );
    debug_char('\r');
    debug_char('\n');
}

void debug_val (const char* label, u4_t val) {
    debug_str(label);
    debug_uint(val);
    debug_char('\r');
    debug_char('\n');
}

void debug_buf (const char* const label, const u1_t* buf, int len)
{
    debug_str(label);
    debug_char(':');
    debug_char(' ');

    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
    debug_char('\r');
    debug_char('\n');
}

void debug_uint (u4_t v) {
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
}

void debug_hex (u1_t b) {
    debug_char("0123456789abcdef"[b>>4]);
    debug_char("0123456789abcdef"[b&0xF]);
}

