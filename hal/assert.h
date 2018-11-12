
#define MBED_ASSERT(expr)  if (!(expr)) { for(;;) asm("nop"); }
