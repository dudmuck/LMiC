#include "hal.h"
#include "device.h"
#include "oslmic.h"

#define UNIT_ID_BASE        FLASH_EEPROM_BASE
#if defined(JOINEUI) && defined(ROOT_APPKEY)
    #define UNIT_ID_LENGTH      8
    #define DEVNONCE_BASE       (UNIT_ID_BASE + UNIT_ID_LENGTH)
    #define DEVNONCE_END       (DEVNONCE_BASE + 16) /* eight uint16_t values */
    #define JOINNONCE_BASE      DEVNONCE_END
    #define JOINNONCE_END      (JOINNONCE_BASE + 32) /* eight uint32_t values */
    #define RJCOUNT1_BASE       JOINNONCE_END
    #define RJCOUNT1_END       (RJCOUNT1_BASE + 16) /* eight uint16_t values */
    #define END                 RJCOUNT1_END
#elif defined(DEVADDR)
    #define UNIT_ID_LENGTH      4
    #define FCNTUP_BASE        (UNIT_ID_BASE + UNIT_ID_LENGTH)
    #define FCNTUP_END         (FCNTUP_BASE + 32)   /* eight uint32_t values */
    #define NFCNTDOWN_BASE      FCNTUP_END
    #define NFCNTDOWN_END      (NFCNTDOWN_BASE + 32)   /* eight uint32_t values */
    #define AFCNTDOWN_BASE      NFCNTDOWN_END
    #define AFCNTDOWN_END      (AFCNTDOWN_BASE + 32)   /* eight uint32_t values */
    #define END                 AFCNTDOWN_END
#else
    #undef UNIT_ID_LENGTH
#endif

void nvm_init()
{
#ifdef UNIT_ID_LENGTH
    HAL_StatusTypeDef status;
    u1_t unitID[8];
    uint32_t* u32ptr;
    uint32_t* startAddr;
    const uint8_t* eePtr;
#if defined(JOINEUI) && defined(ROOT_APPKEY)
    /* OTA */
    os_getDevEui(unitID);
    startAddr = (uint32_t*)DEVNONCE_BASE;
#elif defined(DEVADDR)
    /* ABP */
    u32ptr = (uint32_t*)unitID;
    *u32ptr = DEVADDR;
    startAddr = (uint32_t*)FCNTUP_BASE;
#endif

    eePtr = (void*)UNIT_ID_BASE;
    if (memcmp(eePtr, unitID, UNIT_ID_LENGTH) != 0) {
        unsigned i;
        /* newly (re)provisioned end-device: erase nvm */
        printf("nvm unitID misMatch");
        for (i = 0; i < UNIT_ID_LENGTH; i++)
            printf("%02x ", *eePtr++);
        printf("\r\n");
        if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
            for (;;) asm("nop");
        }

        for (u32ptr = startAddr; u32ptr < (uint32_t*)END; u32ptr++) {
            printf("at %08lx: %08lx   ", (uint32_t)u32ptr, *u32ptr);
            if (*u32ptr != 0) {
                printf("erase ");
                status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, (uint32_t)u32ptr, 0);
                if (status != HAL_OK) {
                    if (status == HAL_ERROR) {
                        for (;;) asm("nop");
                    } else {
                        for (;;) asm("nop");
                    }
                }
            }
            printf("\r\n");
        }
        printf("\r\n");

        /* check if was really cleared */
        for (u32ptr = startAddr; u32ptr < (uint32_t*)END; u32ptr++) {
            printf("At %08lx: %08lx\r\n", (uint32_t)u32ptr, *u32ptr);
            if (*u32ptr != 0) {
                for (;;) asm("nop");
            }
        }

        for (i = 0; i < UNIT_ID_LENGTH; i++) {
            uint32_t destAddr = UNIT_ID_BASE + i;
            status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, destAddr, unitID[i]);
            if (status != HAL_OK) {
                if (status == HAL_ERROR) {
                    for (;;) asm("nop");
                } else {
                    for (;;) asm("nop");
                }
            }
        }


        if (HAL_FLASHEx_DATAEEPROM_Lock() != HAL_OK) {
            for (;;) asm("nop");
        }
    } else {
        printf("nvm unitID match\r\n");
    }
#endif /* UNIT_ID_LENGTH */
}

#if defined(JOINEUI) && defined(ROOT_APPKEY)
static uint16_t eeprom_read_halfword(uint32_t baseAddr, uint32_t endAddr)
{
    uint16_t* u16_ptr = (uint16_t*)baseAddr;
    uint16_t max_val = 0;
    while (u16_ptr < (uint16_t*)endAddr) {
        if (*u16_ptr > max_val)
            max_val = *u16_ptr;

        u16_ptr++;
    }
    return max_val;
}
#endif /* JOINEUI && ROOT_APPKEY */

#ifdef UNIT_ID_LENGTH
static uint32_t eeprom_read_word(uint32_t baseAddr, uint32_t endAddr)
{
    uint32_t* u32_ptr = (uint32_t*)baseAddr;
    uint32_t max_val = 0;
    while (u32_ptr < (uint32_t*)endAddr) {
        //printf("erw %p %lu\r\n", u32_ptr, *u32_ptr);
        if (*u32_ptr > max_val)
            max_val = *u32_ptr;

        u32_ptr++;
    }
    //printf(" read max %lu\r\n", max_val);
    return max_val;
}

u4_t nvm_read(nvm_e nv)
{
    switch (nv) {
#if defined(JOINEUI) && defined(ROOT_APPKEY)
        case NVM_DEVNONCE:
            return eeprom_read_halfword(DEVNONCE_BASE, DEVNONCE_END);
        case NVM_RJCOUNT1:
            return eeprom_read_halfword(RJCOUNT1_BASE, RJCOUNT1_END);
        case NVM_JOINNONCE:
            return eeprom_read_word(JOINNONCE_BASE, JOINNONCE_END);
#elif defined(DEVADDR)
        case NVM_FCNTUP:
            return eeprom_read_word(FCNTUP_BASE, FCNTUP_END);
        case NVM_NFCNTDOWN:
            return eeprom_read_word(NFCNTDOWN_BASE, NFCNTDOWN_END);
        case NVM_AFCNTDOWN:
            return eeprom_read_word(AFCNTDOWN_BASE, AFCNTDOWN_END);
#endif /* */
    }
    for (;;) asm("nop");
}

static void _getmax_word(uint32_t baseAddr, uint32_t endAddr, uint32_t* max_val_at, uint32_t* max_val)
{
    uint32_t* u32_ptr = (uint32_t*)baseAddr;

    while (u32_ptr < (uint32_t*)endAddr) {
        if (*u32_ptr > *max_val) {
            *max_val = *u32_ptr;
            *max_val_at = (uint32_t)u32_ptr;
        }

        u32_ptr++;
    }

    if (*max_val_at == 0) {
        /* first time */
        *max_val_at = baseAddr;
    } else {
        *max_val_at += sizeof(uint32_t);
        if (*max_val_at == endAddr)
            *max_val_at = baseAddr;
    }
}

static void _getmax_halfword(uint32_t baseAddr, uint32_t endAddr, uint32_t* _max_val_at, uint32_t* max_val)
{
    uint16_t* u16_ptr = (uint16_t*)baseAddr;
    uint16_t* max_val_at = NULL;

    while (u16_ptr < (uint16_t*)endAddr) {
        if (*u16_ptr > *max_val) {
            *max_val = *u16_ptr;
            max_val_at = u16_ptr;
        }

        u16_ptr++;
    }

    if (max_val_at == NULL) {
        /* first time */
        max_val_at = (uint16_t*)baseAddr;
    } else {
        if (++max_val_at == (uint16_t*)endAddr)
            max_val_at = (uint16_t*)baseAddr;
    }

    *_max_val_at = (uint32_t)max_val_at;
}

static int increment_eeprom(uint32_t type, uint32_t baseAddr, uint32_t endAddr, uint32_t by)
{
    //bool erased;
    HAL_StatusTypeDef status;
    uint32_t max_val_at = 0;
    uint32_t max_val = 0;
    int i;

    if (type == FLASH_TYPEPROGRAMDATA_HALFWORD)
         _getmax_halfword(baseAddr, endAddr, &max_val_at, &max_val);
    else if (type == FLASH_TYPEPROGRAMDATA_WORD)
         _getmax_word(baseAddr, endAddr, &max_val_at, &max_val);
    else
        return -1;
    /* max_val_at now points to oldest */
    //printf("ie at %08lx max %lu ", max_val_at, max_val);
    //printf("(INCR %lu %lu) ", max_val, by);

    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
        return -2;

    for (i = 0; i < 10; ) {
        status = HAL_FLASHEx_DATAEEPROM_Program(type, max_val_at, max_val + by);
        if (status != HAL_OK) {
            if (i == 9 || status != HAL_ERROR) {
                HAL_FLASHEx_DATAEEPROM_Lock();
                return -4;
            }
        } else
            break;
        if (++i == 10)
            return -6;
    }
    //printf(" -> %lu + %lu\r\n", max_val, by);

    if (HAL_FLASHEx_DATAEEPROM_Lock() != HAL_OK)
        return -5;

    return 0;
}

void nvm_incr(nvm_e nv, uint32_t by)
{
    switch (nv) {
#if defined(JOINEUI) && defined(ROOT_APPKEY)
        case NVM_DEVNONCE:
            increment_eeprom(FLASH_TYPEPROGRAMDATA_HALFWORD, DEVNONCE_BASE, DEVNONCE_END, by);
            break;
        case NVM_RJCOUNT1:
            increment_eeprom(FLASH_TYPEPROGRAMDATA_HALFWORD, RJCOUNT1_BASE, RJCOUNT1_END, by);
            break;
        case NVM_JOINNONCE:
            increment_eeprom(FLASH_TYPEPROGRAMDATA_WORD, JOINNONCE_BASE, JOINNONCE_END, by);
            break;
#elif defined(DEVADDR)
        case NVM_FCNTUP:
            //printf("incFCntUp ");
            increment_eeprom(FLASH_TYPEPROGRAMDATA_WORD, FCNTUP_BASE, FCNTUP_END, by);
            break;
        case NVM_NFCNTDOWN:
            //printf("incNFCntDown ");
            increment_eeprom(FLASH_TYPEPROGRAMDATA_WORD, NFCNTDOWN_BASE, NFCNTDOWN_END, by);
            break;
        case NVM_AFCNTDOWN:
            //printf("incAFCntDown ");
            increment_eeprom(FLASH_TYPEPROGRAMDATA_WORD, AFCNTDOWN_BASE, AFCNTDOWN_END, by);
            break;
#endif /* */
    }
}
#endif /* UNIT_ID_LENGTH */

void nvm_service() { }
