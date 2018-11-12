#include <stdint.h>

void lorawan_app_init(void);
void lorawan_app_mainloop(void);
//extern volatile uint32_t txemptyCnt;
void _ble_nus_data_send(uint8_t* buf, uint8_t len);
//extern uint8_t AppData[];
//extern uint8_t AppDataSize;
//bool SendFrame(void);
