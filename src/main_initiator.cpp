#include "dw3000.h"

#define APP_NAME "UWB DW3000 Initiator"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define RNG_DELAY_MS 13
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
// #define POLL_TX_TO_RESP_RX_DLY_UUS 240
// #define RESP_RX_TIMEOUT_UUS 400

/* delay between frames, in UWB microseconds.
Poll Transmission to Response Reception Delay in UWB Microseconds
This constant defines the delay between the end of the transmission of a "Poll"
message by the initiator and the start of the listening period for a response
message from the responder. It is essential for managing the time between when a
device sends a message and when it expects to start receiving a reply.
This value is used in the function dwt_setrxaftertxdelay() which configures the
DW3000 device to automatically enable its receiver a specified number of
microseconds after it has finished transmitting the poll message. The timing
needs to be precise to ensure that the receiver turns on just before the
expected arrival of the response message, minimizing the receiver's active time
to save power and reduce noise.
*/
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
// #define POLL_TX_TO_RESP_RX_DLY_UUS 1720

/* Response Reception Timeout in UWB Microseconds
This constant specifies the maximum time the receiver will wait for a response
after the poll message's transmission has concluded. If a response is not
received within this interval, a timeout event occurs, indicating a
communication failure or excessive delay.
it ensures that the device does not wait indefinitely for a response that might
have been lost or corrupted during transmission.
 */
// #define RESP_RX_TIMEOUT_UUS 350
#define RESP_RX_TIMEOUT_UUS 1720

/* Default communication configuration. We use default non-STS DW mode. */
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

static uint8_t tx_msg[] = {0x41, 0x88, 0,   0xCA, 0xDE, 'W',
                                'A',  'V',  'E', 0xE0, 0,    0};
static uint8_t rx_msg[] = {0x41, 0x88, 0,    0xCA, 0xDE, 'V', 'E',
                                'W',  'A',  0xE1, 0,    0,    0,   0,
                                0,    0,    0,    0,    0,    0};

/* frame sequence number, incremented after each transmission */
static uint8_t frame_seq_nb = 0;
/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to
 * handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];
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

    delay(2);  // Time needed for DW3000 to start up (transition from INIT_RC to
               // IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc())  // Need to make sure DW IC is in IDLE_RC before
                                // proceeding
    {
        UART_puts("IDLE FAILED\r\n");
        while (1);
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        UART_puts("INIT FAILED\r\n");
        while (1);
    }

    // Enabling LEDs here for debug so that for each TX the D1 LED will flash on
    // DW3000 red eval-shield boards.
    // dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    dwt_setleds(DWT_LEDS_DISABLE);

    /* Configure DW IC. See NOTE 6 below. */
    if (dwt_configure(&config))  // if the dwt_configure returns DWT_ERROR
                                 // either the PLL or RX calibration has failed
                                 // the host should reset the device
    {
        UART_puts("CONFIG FAILED\r\n");
        while (1);
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same
     * delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and
     * also TX/RX LEDs Note, in real low power applications the LEDs should not
     * be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("Range Initiator");
    Serial.println("Setup over........");
}

void loop() {
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    tx_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_msg), tx_msg,
                    0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg), 0,
                     1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that
     * reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* We assume that the transmission is achieved correctly, poll for reception
     * of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
    };

    /* Increment frame sequence number after transmission of the poll message
     * (modulo 256). */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len;

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            /* Check that the frame is the expected response from the companion
             * "SS TWR responder" example. As the sequence number field of the
             * frame is not relevant, it is cleared to simplify the validation
             * of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_msg, ALL_MSG_COMMON_LEN) == 0) {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                float clockOffsetRatio;

                /* Retrieve poll transmission and response reception timestamps.
                 * See NOTE 9 below. */
                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();

                /* Read carrier integrator value and calculate clock offset
                 * ratio. See NOTE 11 below. */
                clockOffsetRatio =
                    ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

                /* Get timestamps embedded in response message. */
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX],
                                &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX],
                                &resp_tx_ts);

                /* Compute time of flight and distance, using clock offset ratio
                 * to correct for differing local and remote clock rates */
                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;

                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) *
                      DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;

                // dwt_starttx(DWT_START_TX_IMMEDIATE);
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
            }
        }
        // delayMicroseconds(600);
        Sleep(RNG_DELAY_MS);
    }
    else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
    /* Execute a delay between ranging exchanges. */
}
