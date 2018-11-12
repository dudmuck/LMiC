#include "hal.h"
#include "nrf_gpio.h"

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void hal_pin_antsw(s1_t val)
{
    if (val > 0)
        NRF_GPIO->OUTSET = (1 << RXTX_PIN);
    else
        NRF_GPIO->OUTCLR = (1 << RXTX_PIN);
}
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

void hal_board_io_init()
{
#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    #ifdef TARGET_FF_ARDUINO
        nrf_gpio_cfg_input(RXTX_PIN, NRF_GPIO_PIN_NOPULL);
        if ((NRF_GPIO->IN >> RXTX_PIN) & 0x1UL) 
            txpin = TX_PIN_PABOOST; // shield LAS board
        else
            txpin = TX_PIN_RFO; // shield MAS board

        nrf_gpio_cfg_output(RXTX_PIN);
    #else
        #error implement_non_arduino_target
    #endif
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */
}
