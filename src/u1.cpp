#include "dw3000.h"
#include "general.h"

#define APP_NAME "UWB DW3000 U1"
#define UID U1

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

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

static uint8_t tx_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, UID, 0, FUNC_CODE_INTER,
    0,    0,    0, 0,    0,    0, 0, 0,   0, 0};
static uint8_t rx_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_CODE_INTER,
    0,    0,    0, 0,    0,    0, 0, 0, 0, 0,
    0,    0,    0, 0,    0,    0, 0, 0, 0, 0,
    0,    0};

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[NUM_NODES - 1][RX_BUF_LEN];
static uint32_t status_reg = 0;
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

    Serial.print(UID);
    Serial.println(": setup over");
}

uint32_t poll_tx_ts, poll_rx_ts, resp_tx_ts;
uint32_t resp_rx_ts[NUM_NODES - 1] = {0};
uint32_t rtd_init, rtd_resp;

int counter = 0;
int preUID = 0;

void loop() {
    if (counter == 0) {
        // dwt_setrxtimeout(RX_TIMEOUT_UUS);
        tx_msg[MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
        dwt_writetxfctrl(sizeof(tx_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        // dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
    };

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        dwt_readrxdata(rx_buffer[counter], frame_len, 0);
        resp_rx_ts[counter] = dwt_readrxtimestamplo32();
        if (rx_buffer[counter][MSG_SID_IDX] <= preUID) {
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            counter = 0;
            preUID = 0;
            return;
        }
        preUID = rx_buffer[counter][MSG_SID_IDX];
        ++counter;
    } else {
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        counter = 0;
        preUID = 0;
        return;
    }
    if (counter == NUM_NODES - 1) {
        /* calculate distance */
        float clockOffsetRatio;
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
        poll_tx_ts = dwt_readtxtimestamplo32();
        currentDebugMillis = millis();
        Serial.print(currentDebugMillis - previousDebugMillis);
        Serial.print("ms\t");
        for (int i = 0; i < counter; i++) {
            if ((rx_buffer[i][MSG_FUNC_IDX] == FUNC_CODE_INTER)) {
                // resp_msg_get_ts(&rx_buffer[i][MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                // resp_msg_get_ts(&rx_buffer[i][MSG_RESP_TX_TS_IDX], &resp_tx_ts);
                resp_msg_get_ts(&rx_buffer[i][MSG_TO_1], &rtd_resp);
                rtd_init = resp_rx_ts[i] - poll_tx_ts;
                // rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) *
                      DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
                Serial.print(rx_buffer[i][MSG_SID_IDX]);
                Serial.print("\t");
                Serial.print(rx_buffer[i][MSG_SN_IDX]);
                Serial.print("\t");
                snprintf(dist_str, sizeof(dist_str), "%3.2f m\t", distance);
                Serial.print(dist_str);
            }
        }
        Serial.print("\t");
        Serial.print(frame_seq_nb);
        Serial.println();
        previousDebugMillis = currentDebugMillis;
        counter = 0;
        preUID = 0;
        ++frame_seq_nb;
        Sleep(INTERVAL);
    }
}
