#include "lmic.h"
#include <stdint.h>

#define UP_FPORT        15

void lorawan_app_init(void);
void lorawan_app_mainloop(void);
void _ble_nus_data_send(uint8_t* buf, uint8_t len);

