#include "types.h"
#include "utilities.h"

void memcpyr( u1_t *dst, const u1_t *src, u2_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {   
        *dst-- = *src++;
    }
}

