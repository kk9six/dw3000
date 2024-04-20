#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define NUM_NODES 6
#define MAX_NODE_NUM 6

#define FUNC_CODE_POLL 0xE0
#define FUNC_CODE_RESP 0xE1
#define FUNC_CODE_INTER 0xE2

#define U0 0x36

#define U1 0x30
#define U2 0x31
#define U3 0x32
#define U4 0x33
#define U5 0x34
#define U6 0x35

#define INTERVAL 3 /* FPS = 1003 / INTERVAL */
#define MSG_COMMON_LEN 5
#define MSG_SN_IDX 2   /* sequence number */
#define MSG_SID_IDX 7  /* source id */
#define MSG_FUNC_IDX 9 /* func code*/

#define MSG_TO_1 10
#define MSG_TO_2 14
#define MSG_TO_3 18
#define MSG_TO_4 22
#define MSG_TO_5 26
#define MSG_TO_6 30

#define MSG_POLL_RX_TS_IDX 10
#define MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define TX_TO_RX_DLY_UUS 100
#define RX_TIMEOUT_UUS 20000

#define RX_BUF_LEN 32
