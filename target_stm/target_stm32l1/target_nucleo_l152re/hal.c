#include "hal.h"

/* radio chip on arduino shield */

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void hal_pin_antsw(s1_t val)
{
    HAL_GPIO_WritePin(RADIO_RXTX_PORT, RADIO_RXTX_PIN, val > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

void hal_board_io_init()
{
#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = RADIO_RXTX_PIN;
    RADIO_RXTX_PORT_ENABLE;
    HAL_GPIO_Init(RADIO_RXTX_PORT, &GPIO_InitStruct);
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

#ifdef TARGET_FF_ARDUINO
    #if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
        if (HAL_GPIO_ReadPin(RADIO_RXTX_PORT, RADIO_RXTX_PIN) == GPIO_PIN_SET)
            txpin = TX_PIN_PABOOST; // shield LAS board
        else
            txpin = TX_PIN_RFO; // shield MAS board
    #endif
#else
    #error implement_non_arduino_target
#endif

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(RADIO_RXTX_PORT, &GPIO_InitStruct);
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */
}
