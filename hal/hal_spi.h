#include "oslmic.h"

void hal_spi_init(void);

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void writeReg (u1_t addr, u1_t data );
u1_t readReg (u1_t addr);
void writeBuf (u1_t addr, xref2u1_t buf, u1_t len);
void readBuf (u1_t addr, xref2u1_t buf, u1_t len);

#elif defined(CFG_sx126x_radio)

void radio_xfer(uint8_t opcode, uint8_t wlen, uint8_t rlen, uint8_t* ptr);
void readBuf(uint8_t size, uint8_t offset, uint8_t* out);
void writeBuf(uint8_t size, uint8_t offset, uint8_t* in);

#endif
