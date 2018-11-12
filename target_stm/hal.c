
#include "hal.h"
#include "hal_spi.h"
#include <string.h>

void getHWDevEui(u1_t* devEui)
{
    u4_t id;
    u1_t* u1ptr = (u1_t*)&id;

    memset(devEui, 0, 8);

    id = LL_GetUID_Word0();
    devEui[0] ^= u1ptr[0];
    devEui[1] ^= u1ptr[1];
    devEui[2] ^= u1ptr[2];
    devEui[3] ^= u1ptr[3];

    id = LL_GetUID_Word1();
    devEui[2] ^= u1ptr[0];
    devEui[3] ^= u1ptr[1];
    devEui[4] ^= u1ptr[2];
    devEui[5] ^= u1ptr[3];

    id = LL_GetUID_Word2();
    devEui[4] ^= u1ptr[0];
    devEui[5] ^= u1ptr[1];
    devEui[6] ^= u1ptr[2];
    devEui[7] ^= u1ptr[3];
}

static GPIO_InitTypeDef GPIO_InitStruct_rst;

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)

void hal_pin_rst(u1_t val)
{
    if(val == 0 || val == 1) { // drive pin
        GPIO_InitStruct_rst.Mode  = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(RADIO_RESET_PORT, &GPIO_InitStruct_rst);
        HAL_GPIO_WritePin(RADIO_RESET_PORT, RADIO_RESET_PIN, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else { // keep pin floating
        GPIO_InitStruct_rst.Mode  = GPIO_MODE_INPUT;
        HAL_GPIO_Init(RADIO_RESET_PORT, &GPIO_InitStruct_rst);
    }
}
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

#if defined(CFG_sx126x_radio)
void hal_pin_antsw (u1_t val)
{
    HAL_GPIO_WritePin(RADIO_ANTSW_PORT, RADIO_ANTSW_PIN, val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#endif /* CFG_sx126x_radio */

void hal_dbg_pin_rx(bool hi)
{
    HAL_GPIO_WritePin(RXDBG_PORT, RXDBG_PIN, hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hal_dbg_pin_latency(bool hi)
{
    HAL_GPIO_WritePin(LATE_PORT, LATE_PIN, hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


#if defined(CFG_sx126x_radio)
void hal_sx126x_init(void)
{
    //printf("hal_sx126x_init ");
    if (HAL_GPIO_ReadPin(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == GPIO_PIN_SET) {
        //printf("radio_busy\r\n");
        GPIO_InitStruct_rst.Mode  = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(RADIO_RESET_PORT, &GPIO_InitStruct_rst);
        HAL_GPIO_WritePin(RADIO_RESET_PORT, RADIO_RESET_PIN, GPIO_PIN_RESET);
        wait_ms(20);
        GPIO_InitStruct_rst.Mode  = GPIO_MODE_INPUT;
        HAL_GPIO_Init(RADIO_RESET_PORT, &GPIO_InitStruct_rst);
        while (HAL_GPIO_ReadPin(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == GPIO_PIN_SET)
            asm("nop");

        wait_ms(10);

        while (HAL_GPIO_ReadPin(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == GPIO_PIN_SET)
            asm("nop");
        //printf("unBusyd\r\n");
    } /*else
        printf("notBusy\r\n");*/
}
#endif /* CFG_sx126x_radio */

void hal_io_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct_rst.Pull  = GPIO_NOPULL;
    GPIO_InitStruct_rst.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct_rst.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_rst.Pin = RADIO_RESET_PIN;
    RADIO_RESET_PORT_ENABLE;
    HAL_GPIO_Init(RADIO_RESET_PORT, &GPIO_InitStruct_rst);
    
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;

#ifdef TARGET_FF_ARDUINO
    /* radio chip on arduino shield */
    #if defined(CFG_sx126x_radio)
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pin = RADIO_BUSY_PIN;
        HAL_GPIO_Init(RADIO_BUSY_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RADIO_XTAL_SEL_PIN;
        HAL_GPIO_Init(RADIO_XTAL_SEL_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RADIO_DEVICE_SEL_PIN;
        HAL_GPIO_Init(RADIO_DEVICE_SEL_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;

        GPIO_InitStruct.Pin = TX_LED_PIN;
        HAL_GPIO_Init(TX_LED_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RX_LED_PIN;
        HAL_GPIO_Init(RX_LED_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RADIO_ANTSW_PIN;
        HAL_GPIO_Init(RADIO_ANTSW_PORT, &GPIO_InitStruct);
    #endif
#else
    /* implement_non_arduino_target */
#endif

    RXDBG_PORT_ENABLE;
    
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = RXDBG_PIN;
    HAL_GPIO_Init(RXDBG_PORT, &GPIO_InitStruct);

    LATE_PORT_ENABLE;

    GPIO_InitStruct.Pin = LATE_PIN;
    HAL_GPIO_Init(LATE_PORT, &GPIO_InitStruct);

    /***************** EXTI DIO *****************/
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    GPIO_InitStruct.Pin = DIO0_PIN;
    DIO0_PORT_ENABLE;
    HAL_GPIO_Init(DIO0_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(DIO0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DIO0_IRQn);
#endif

    DIO1_PORT_ENABLE;
    GPIO_InitStruct.Pin = DIO1_PIN;
    HAL_GPIO_Init(DIO1_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(DIO1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DIO1_IRQn);
} // ..hal_io_init()

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void DIO0_IRQHandler()
{
    HAL_GPIO_EXTI_IRQHandler(DIO0_PIN);
}
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

void DIO1_IRQHandler()
{
    HAL_GPIO_EXTI_IRQHandler(DIO1_PIN);
}

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
extern void radio_irq_handler (u1_t dio);
#elif defined(CFG_sx126x_radio)
extern void radio_irq_handler (void);
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    radio_irq_handler(GPIO_Pin == DIO1_PIN ? 1 : 0);
#elif defined(CFG_sx126x_radio)
    radio_irq_handler();
#else
    #error radio_irq_handler
#endif
}

void mbed_sdk_init(void);

void hal_board_io_init(void);

void _hal_init()
{
    mbed_sdk_init();

    hal_io_init();
    hal_board_io_init();

    hal_spi_init();

    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_DBGMCU_EnableDBGStandbyMode();
}

