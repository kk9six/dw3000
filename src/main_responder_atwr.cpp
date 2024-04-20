#include "SPI.h"
#include "dw3000.h"

#define APP_NAME "UWB DW3000 Responder"

extern SPISettings _fastSPI;

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define FUNC_CODE_POLL 0xE0
#define FUNC_CODE_ACK 0xE1
#define FUNC_CODE_RANGE 0xE2
#define FUNC_CODE_FINAL 0xE3

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define RESP_MSG_TS_LEN 4

#define TX_TO_RX_DLY_UUS 100
#define RX_TO_TX_DLY_UUS 600
#define RX_TIMEOUT_UUS 1720

static dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for
          non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e
                       */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t poll_msg[] = {0x41, 0x88, 0,   0xCA,           0xDE, 'W',
                             'A',  'V',  'E', FUNC_CODE_POLL, 0,    0};
static uint8_t ack_msg[] = {0x41, 0x88,          0, 0xCA, 0xDE, 'V', 'E', 'W',
                            'A',  FUNC_CODE_ACK, 0, 0,    0,    0,   0,   0};
static uint8_t range_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', FUNC_CODE_RANGE, 0, 0};
static uint8_t final_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', FUNC_CODE_FINAL,
    0,    0,    0, 0,    0,    0,   0,   0,   0,   0};

#define ALL_MSG_SN_IDX 2
#define POLL_RX_TS_IDX 10
#define ACK_TX_TS_IDX 14
#define RANGE_RX_TS_IDX 10

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];

static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t ack_tx_ts;
static uint64_t range_rx_ts;

bool has_poll;
bool sent_ack;
int ret;

extern dwt_txconfig_t txconfig_options;

void setup() {
    UART_init();

    // _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);

    delay(2);
    while (!dwt_checkidlerc()) {
        UART_puts("IDLE FAILED\r\n");
        while (1);
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        UART_puts("INIT FAILED\r\n");
        while (1);
    }

    dwt_setleds(DWT_LEDS_DISABLE);

    if (dwt_configure(&config)) {
        UART_puts("CONFIG FAILED\r\n");
        while (1);
    }
    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(TX_TO_RX_DLY_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("Range Responder");
    Serial.println("Setup over........");
}

void loop() {
    if (!has_poll) {
        /* waiting for poll msg, disable timeout */
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        };
    } else { /* has poll, waiting for the range msg */
        dwt_setrxtimeout(RX_TIMEOUT_UUS);
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
                  SYS_STATUS_ALL_RX_ERR))) {
        };
        frame_seq_nb++;
    }
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, poll_msg, ALL_MSG_COMMON_LEN) == 0) {
                if (has_poll) {
                    has_poll = false;
                    return;
                }
                poll_rx_ts = get_rx_timestamp_u64();
                ack_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                resp_msg_set_ts(&ack_msg[POLL_RX_TS_IDX], poll_rx_ts);
                dwt_writetxdata(sizeof(ack_msg), ack_msg, 0);
                dwt_writetxfctrl(sizeof(ack_msg), 0, 1);
                ret =
                    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
                if (ret == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                             SYS_STATUS_TXFRS_BIT_MASK)) {
                    };
                    has_poll = true;
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
            } else {
                ack_tx_ts = get_tx_timestamp_u64();
                range_rx_ts = get_rx_timestamp_u64();
                final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                resp_msg_set_ts(&final_msg[ACK_TX_TS_IDX], ack_tx_ts);
                resp_msg_set_ts(&final_msg[RANGE_RX_TS_IDX], range_rx_ts);
                dwt_writetxdata(sizeof(final_msg), final_msg, 0);
                dwt_writetxfctrl(sizeof(final_msg), 0, 1);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
                if (ret == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                             SYS_STATUS_TXFRS_BIT_MASK)) {
                    };
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
                has_poll = false;
                frame_seq_nb++;
            }
        }
    } else {
        has_poll = false;
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    }
}
