#include "hal.h"
#include "device.h"
#include "oslmic.h"

#include "nrf_log.h"
#include "nrf_fstorage.h"
#include "nrf_soc.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"
#endif

#include "app_error.h"

#define BLANK_ID        0xffffffff
#define FCNTUP_ID       0xfffffffe
#define NFCNTDOWN_ID    0xfffffffd
#define AFCNTDOWN_ID    0xfffffffc
#define DEVNONCE_ID     0xfffffffb
#define RJCOUNT1_ID     0xfffffffa
#define JOINNONCE_ID    0xfffffff9

volatile uint32_t incr_id;
volatile uint32_t incr_by;

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x7f000,
    .end_addr   = 0x7ffff,
};

#define UNIT_ID_BASE        fstorage.start_addr
#if defined(JOINEUI) && defined(ROOT_APPKEY)
    /* OTA lorawan-1.1 */
    #define UNIT_ID_LENGTH      8
#elif defined(DEVADDR)
    /* ABP */
    #define UNIT_ID_LENGTH      4
#else
    #undef UNIT_ID_LENGTH
#endif

static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

volatile uint32_t endAddr;

#ifdef UNIT_ID_LENGTH
/* return true for failure */
static bool checkIDs()
{
    bool ret = false;
    uint32_t a;
    uint32_t* u32ptr;

    for (a = fstorage.start_addr + UNIT_ID_LENGTH; a < fstorage.end_addr && !ret;) {
        u32ptr = (uint32_t*)a;
        switch (*u32ptr) {
            case BLANK_ID:
                a += 4;
                continue;
            case FCNTUP_ID:
            case NFCNTDOWN_ID:
            case AFCNTDOWN_ID:
            case DEVNONCE_ID:
            case RJCOUNT1_ID:
            case JOINNONCE_ID:
                a += 8;
                break;
            default:
                printf("->%x ", *u32ptr);
                printf("bad at %x\r\n", a);
                ret = true;
                break;
        } // ..switch (*u32ptr)
    } // ..for()

    return ret;
}
#endif /* UNIT_ID_LENGTH */

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
#ifdef SOFTDEVICE_PRESENT
        (void) sd_app_evt_wait();
#else
        __WFE();
#endif
    }
}

void nvm_init()
{
#ifdef UNIT_ID_LENGTH
    uint32_t a;
    uint32_t* u32ptr;
    u1_t unitID[8];
    uint32_t* startAddr;
    ret_code_t rc;
    nrf_fstorage_api_t * p_fs_api;

#ifdef SOFTDEVICE_PRESENT
    NRF_LOG_INFO("SoftDevice is present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_sd implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
#else
    NRF_LOG_INFO("SoftDevice not present.");
    NRF_LOG_INFO("Initializing nrf_fstorage_nvmc implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
     * nrf_fstorage_nvmc uses the NVMC peripheral. This implementation can be used when the
     * SoftDevice is disabled or not present.
     *
     * Using this implementation when the SoftDevice is enabled results in a hardfault. */
    p_fs_api = &nrf_fstorage_nvmc;
#endif

    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    endAddr = nrf5_flash_end_addr_get();

#if defined(JOINEUI) && defined(ROOT_APPKEY)
    /* OTA */
    os_getDevEui(unitID);
    startAddr++;
#elif defined(DEVADDR)
    /* ABP */
    u32ptr = (uint32_t*)unitID;
    *u32ptr = DEVADDR;
#endif
    startAddr++;

    if (memcmp((void*)fstorage.start_addr, unitID, UNIT_ID_LENGTH != 0) || checkIDs()) {
        printf("unitID mismatch, erase\r\n");
        rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
        APP_ERROR_CHECK(rc);
        wait_for_flash_ready(&fstorage);

        printf("flashErase done\r\n");
        rc = nrf_fstorage_write(&fstorage, fstorage.start_addr, unitID, UNIT_ID_LENGTH, NULL);
        APP_ERROR_CHECK(rc);
        wait_for_flash_ready(&fstorage);
        printf("written: ");
        {
            u1_t* u1ptr = (u1_t*)fstorage.start_addr;
            for (unsigned n = 0; n < UNIT_ID_LENGTH; n++)
                printf("%02x ", u1ptr[n]);
            printf("\r\n");
        }
    } // ..erase

    incr_id = BLANK_ID;

#endif /* UNIT_ID_LENGTH */
}

volatile uint32_t badAddr;

#ifdef UNIT_ID_LENGTH
static u4_t _read_word(u4_t id)
{
    uint32_t a;
    uint32_t* u32ptr;
    uint32_t maxval = 0;

    for (a = fstorage.start_addr + UNIT_ID_LENGTH; a < fstorage.end_addr;) {
        u32ptr = (uint32_t*)a;
        switch (*u32ptr) {
            case BLANK_ID:
                a += 4;
                continue;
            case FCNTUP_ID:
            case NFCNTDOWN_ID:
            case AFCNTDOWN_ID:
            case DEVNONCE_ID:
            case RJCOUNT1_ID:
            case JOINNONCE_ID:
                a += 4;
                if (*u32ptr == id) {
                    u32ptr = (uint32_t*)a;
                    if (*u32ptr > maxval) {
                        maxval = *u32ptr;
                    }
                }
                a += 4;
                break;
            default:
                incr_by = *u32ptr;
                badAddr = a;
                for (;;) asm("nop");
                break;
        } // ..switch (*u32ptr)
    } // ..for()

    return maxval;
}  // .._read_word()

u4_t nvm_read(nvm_e nv)
{
    if (nrf_fstorage_is_busy(&fstorage)) {
        for (;;) asm("nop");
    }

    switch (nv) {
#if defined(JOINEUI) && defined(ROOT_APPKEY)
        case NVM_DEVNONCE:
            return _read_word(DEVNONCE_ID);
        case NVM_RJCOUNT1:
            return _read_word(RJCOUNT1_ID);
        case NVM_JOINNONCE:
            return _read_word(JOINNONCE_ID);
#elif defined(DEVADDR)
        case NVM_FCNTUP:
            return _read_word(FCNTUP_ID);
        case NVM_NFCNTDOWN:
            return _read_word(NFCNTDOWN_ID);
        case NVM_AFCNTDOWN:
            return _read_word(AFCNTDOWN_ID);
#endif /* */
    }

    return 0;
}

void nvm_incr(nvm_e nv, uint32_t by)
{
    incr_by = by;

    switch (nv) {
#if defined(JOINEUI) && defined(ROOT_APPKEY)
        case NVM_DEVNONCE:
            incr_id = DEVNONCE_ID;
            break;
        case NVM_RJCOUNT1:
            incr_id = RJCOUNT1_ID;
            break;
        case NVM_JOINNONCE:
            incr_id = JOINNONCE_ID;
            break;
#elif defined(DEVADDR)
        case NVM_FCNTUP:
            incr_id = FCNTUP_ID;
            break;
        case NVM_NFCNTDOWN:
            incr_id = NFCNTDOWN_ID;
            break;
        case NVM_AFCNTDOWN:
            incr_id = AFCNTDOWN_ID;
            break;
#endif /* */
    }
}
#endif /* UNIT_ID_LENGTH */

static bool _addr_is_within_bounds(nrf_fstorage_t const * p_fs,
                                  uint32_t               addr,
                                  uint32_t               len)
{
    return (   (addr           >= p_fs->start_addr)
            && (addr + len - 1 <= p_fs->end_addr));
}


void nvm_service()
{
#ifdef UNIT_ID_LENGTH
    ret_code_t rc;
    uint32_t* u32ptr;
    uint32_t a;
    u4_t val;
    bool b = false;
    unsigned wlen;
    uint8_t wbuf[8];
    uint32_t endAddr = fstorage.end_addr - 8;

    if (nrf_fstorage_is_busy(&fstorage))
        return;

    if (incr_id == BLANK_ID)
        return;

    /* get max stored value */
    val = _read_word(incr_id);

    /* find lowest unused location */
    for (a = fstorage.start_addr + UNIT_ID_LENGTH; a < endAddr;) {
        u32ptr = (uint32_t*)a;
        switch (*u32ptr) {
            case BLANK_ID:
                b = true;
                //printf("blank at %08x\r\n", a);
                break;
            case FCNTUP_ID:
            case NFCNTDOWN_ID:
            case AFCNTDOWN_ID:
            case DEVNONCE_ID:
            case RJCOUNT1_ID:
            case JOINNONCE_ID:
                a += 8;
                break;
            default:
                for (;;) asm("nop");
        }
        if (b)
            break;
    }

    if (b) {
        /* have available location */
        u32ptr = (uint32_t*)wbuf;
        *u32ptr++ = incr_id;
        *u32ptr = val + incr_by;

        //printf("write %x to %p\r\n", *u32ptr, a);
        if (!_addr_is_within_bounds(&fstorage, a, 8)) {
            for (;;) asm("nop");
        }
        rc = nrf_fstorage_write(&fstorage, a, wbuf, 8, NULL);
        APP_ERROR_CHECK(rc);
        /*for (unsigned n = 0; n < 8; n++)
            printf("%02x ", wbuf[n]);
        printf("\r\n");*/
        u32ptr = (uint32_t*)a;
        wait_for_flash_ready(&fstorage);
        /*printf("1stword:%x\r\n", *u32ptr++);
        printf("2ndword:%x\r\n", *u32ptr++);*/
    } else {
        u1_t unitID[8];
        uint32_t fcntup, nfcntdown, afcntdown;
        /* flash full */
        printf("flashFull\r\n");
        /* get unitID and all max values */
        memcpy(unitID, (void*)fstorage.start_addr, UNIT_ID_LENGTH);
        fcntup = _read_word(FCNTUP_ID);
        nfcntdown = _read_word(NFCNTDOWN_ID);
        afcntdown = _read_word(AFCNTDOWN_ID);
        /* erase page */
        rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
        APP_ERROR_CHECK(rc);
        wait_for_flash_ready(&fstorage);

        /* write back unit ID */
        rc = nrf_fstorage_write(&fstorage, fstorage.start_addr, unitID, UNIT_ID_LENGTH, NULL);
        APP_ERROR_CHECK(rc);
        a = fstorage.start_addr + UNIT_ID_LENGTH;
        wait_for_flash_ready(&fstorage);

        /* write back max values */
        u32ptr = (uint32_t*)wbuf;
        *u32ptr++ = FCNTUP_ID;
        *u32ptr = fcntup;
        if (incr_id == FCNTUP_ID)
            *u32ptr += incr_by;

        rc = nrf_fstorage_write(&fstorage, a, wbuf, 8, NULL);
        APP_ERROR_CHECK(rc);
        a += 8;
        wait_for_flash_ready(&fstorage);

        *u32ptr++ = NFCNTDOWN_ID;
        *u32ptr = nfcntdown;
        if (incr_id == NFCNTDOWN_ID)
            *u32ptr += incr_by;

        rc = nrf_fstorage_write(&fstorage, a, wbuf, 8, NULL);
        APP_ERROR_CHECK(rc);
        a += 8;
        wait_for_flash_ready(&fstorage);

        *u32ptr++ = AFCNTDOWN_ID;
        *u32ptr = afcntdown;
        if (incr_id == AFCNTDOWN_ID)
            *u32ptr += incr_by;

        rc = nrf_fstorage_write(&fstorage, a, wbuf, 8, NULL);
        APP_ERROR_CHECK(rc);
        wait_for_flash_ready(&fstorage);
    }

    incr_id = BLANK_ID;
#endif /* UNIT_ID_LENGTH */
}

