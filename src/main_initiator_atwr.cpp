#include "dw3000.h"

#define APP_NAME "UWB DW3000 Initiator"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define FUNC_CODE_POLL 0xE0
#define FUNC_CODE_ACK 0xE1
#define FUNC_CODE_RANGE 0xE2
#define FUNC_CODE_FINAL 0xE3

#define RNG_DELAY_MS 5
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define RESP_MSG_TS_LEN 4

#define TX_TO_RX_DLY_UUS 100
#define RX_TO_TX_DLY_UUS 600
#define RX_TIMEOUT_UUS 1720

bool has_ack = false;
bool sent_range = false;

float clockOffsetRatioAck;
float clockOffsetRatioFinal;
int ret;

uint64_t poll_tx_ts;
uint32_t poll_rx_ts, ack_tx_ts, range_rx_ts;

static dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2
          for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC
                         size).    Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum
                       * dwt_sts_lengths_e
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
static uint8_t ack_buffer[20];
static uint8_t final_buffer[20];
static uint8_t rx_buffer[20];

static uint32_t status_reg = 0;
static uint64_t ack_rx_ts;
static uint64_t range_tx_ts;
static double t_round_1, t_reply_1, t_round_2, t_reply_2;
static double tof;
static double distance;
extern dwt_txconfig_t txconfig_options;

unsigned long previousDebugMillis = 0;
unsigned long currentDebugMillis = 0;
int millisSinceLastSerialPrint;

void setup() {
    UART_init();
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
    dwt_setrxtimeout(RX_TIMEOUT_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("Range Initiator");
    Serial.println("Setup over........");
}

void loop() {
    if (!has_ack) {
        /* send poll msg */
        poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
        dwt_writetxfctrl(sizeof(poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
                  SYS_STATUS_ALL_RX_ERR))) {
        };
        frame_seq_nb++;
    } else {
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
                  SYS_STATUS_ALL_RX_ERR))) {
        };
        frame_seq_nb++;
    }
    /* receive the ack msg */
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, ack_msg, ALL_MSG_COMMON_LEN) == 0) {
                Serial.println("Response is ACK");
                poll_tx_ts = get_tx_timestamp_u64();
                resp_msg_get_ts(&rx_buffer[POLL_RX_TS_IDX], &poll_rx_ts);
                ack_rx_ts = get_rx_timestamp_u64();
                clockOffsetRatioAck =
                    ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
                /* send range msg */
                range_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_write32bitreg(SYS_STATUS_ID,
                                  SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                dwt_writetxdata(sizeof(range_msg), range_msg, 0);
                dwt_writetxfctrl(sizeof(range_msg), 0, 1);
                ret =
                    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
                if (ret == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                             SYS_STATUS_TXFRS_BIT_MASK)) {
                    };
                    has_ack = true;
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                }
            } else {
                Serial.println("Response is Final");
                range_tx_ts = get_tx_timestamp_u64();
                clockOffsetRatioFinal =
                    ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
                resp_msg_get_ts(&rx_buffer[ACK_TX_TS_IDX], &ack_tx_ts);
                resp_msg_get_ts(&rx_buffer[RANGE_RX_TS_IDX], &range_rx_ts);

                t_round_1 = ack_rx_ts - poll_tx_ts;
                t_round_2 = (range_rx_ts -
                             ack_tx_ts);  // * (1 - clockOffsetRatioFinal);
                t_reply_1 =
                    (ack_tx_ts - poll_rx_ts);  // * (1 - clockOffsetRatioAck);
                t_reply_2 = range_tx_ts - ack_rx_ts;
                tof = (t_round_1 * t_round_2 - t_reply_1 * t_reply_2) /
                      (t_round_1 + t_round_2 + t_reply_1 + t_reply_2) *
                      DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;

                currentDebugMillis = millis();
                // Serial.print("Current ts: ");
                // Serial.print(currentDebugMillis);
                // Serial.print("\t");
                // Serial.print("Since previous ts: ");
                Serial.print("Interval: ");
                Serial.print(currentDebugMillis - previousDebugMillis);
                Serial.print("ms\t");
                /* Display computed distance on LCD. */
                snprintf(dist_str, sizeof(dist_str), "%3.2f m", distance);
                test_run_info((unsigned char *)dist_str);
                previousDebugMillis = currentDebugMillis;
                has_ack = false;
                sent_range = false;
                Sleep(RNG_DELAY_MS);
            }
        }
    } else {
        has_ack = false;
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}
