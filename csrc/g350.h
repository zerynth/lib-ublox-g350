#define MAX_BUF 1024
#define MAX_CMD 545
#define MAX_SOCKS 7
#define MAX_SOCK_HEX_BUF 128
#define MAX_SOCK_HEX_RXBUF 128+32
#define MAX_OPS   6
#define MAX_ERR_LEN 32
#define GS_TIMEOUT 1000
#define GS_TLS_PROFILE 1

typedef struct _gsm_socket {
    uint8_t acquired;
    uint8_t proto;
    uint8_t to_be_closed;
    uint8_t secure;
    uint16_t unused;
    uint16_t timeout;
    VSemaphore rx;
    VSemaphore lock;
    uint8_t txbuf[MAX_SOCK_HEX_BUF];
    uint8_t rxbuf[MAX_SOCK_HEX_RXBUF];
} GSocket;

//COMMANDS

#define MAKE_CMD(group,command,response) (((group)<<24)|((command)<<16)|(response))
#define DEF_CMD(cmd,response,urc,id)  {cmd,sizeof(cmd)-1,response,urc,id}

typedef struct _gs_cmd {
    uint8_t body[16];
    uint8_t len;
    uint8_t response_type;
    uint8_t urc;
    uint8_t id;
} GSCmd;


//COMMAND SLOTS

#define MAX_SLOTS 16
typedef struct _gs_slot {
    GSCmd *cmd;
    uint8_t err;
    uint8_t allocated;
    uint8_t has_params;
    uint8_t params;
    uint16_t max_size;
    uint16_t unused2;
    uint32_t stime;
    uint32_t timeout;
    uint8_t *resp;
    uint8_t *eresp;
} GSSlot;

////////////OPERATORS

typedef struct _gs_operator {
    uint8_t type;
    uint8_t fmtl_l;
    uint8_t fmts_l;
    uint8_t fmtc_l;
    uint8_t fmt_long[24];
    uint8_t fmt_short[10];
    uint8_t fmt_code[6];
}GSOp;


////////////GSM STATUS

typedef struct _gsm_status{
    uint8_t initialized;
    uint8_t attached;
    uint8_t registered;
    int8_t secure_sock_id;
    uint8_t gprs;
    uint8_t gprs_mode;
    uint8_t errlen;
    uint8_t mode;
    uint8_t rssi;
    uint8_t serial;
    uint16_t dtr;
    uint16_t rts;
    uint16_t rx;
    uint16_t tx;
    uint16_t poweron;
    uint16_t reset;
    uint16_t bytes;
    GSSlot *slot;
    VSemaphore sendlock;
    VSemaphore slotlock;
    VSemaphore slotdone;
    VThread thread;
    uint8_t errmsg[MAX_ERR_LEN];
    uint8_t buffer[MAX_CMD];
} GStatus;

//DEFINES
#define GS_PROFILE 0

#define GS_ERR_OK      0
#define GS_ERR_TIMEOUT 1


#define GS_REG_DENIED  2
#define GS_REG_NOT     0
#define GS_REG_OK      1
#define GS_REG_ROAMING 3

#define KNOWN_COMMANDS (sizeof(gs_commands)/sizeof(GSCmd))
#define GS_MIN(a)   (((a)<(gs.bytes)) ? (a):(gs.bytes))

#define GS_MODE_NORMAL 0
#define GS_MODE_PROMPT 1

#define GS_CMD_NORMAL 0
#define GS_CMD_URC    1 
#define GS_CMD_LINE   2

//RESPONSES
// only ok
#define GS_RES_OK        0
// one line of params, then ok
#define GS_RES_PARAM_OK  1
// no answer
#define GS_RES_NO        2



#define GS_CMD_CCID             0
#define GS_CMD_CCLK             1
#define GS_CMD_CGATT            2
#define GS_CMD_CGED             3
#define GS_CMD_CGSN             4
#define GS_CMD_CIEV             5
#define GS_CMD_CMEE             6
#define GS_CMD_CMER             7
#define GS_CMD_COPS             8
#define GS_CMD_CREG             9
#define GS_CMD_IPR              10
#define GS_CMD_UDCONF           11
#define GS_CMD_UDNSRN           12
#define GS_CMD_UPSD             13
#define GS_CMD_UPSDA            14
#define GS_CMD_UPSND            15
#define GS_CMD_URAT             16
#define GS_CMD_USECMNG          17
#define GS_CMD_USECPRF          18
#define GS_CMD_USOCL            19
#define GS_CMD_USOCO            20
#define GS_CMD_USOCR            21
#define GS_CMD_USOCTL           22
#define GS_CMD_USOGO            23
#define GS_CMD_USOLI            24
#define GS_CMD_USORD            25
#define GS_CMD_USORF            26
#define GS_CMD_USOSEC           27
#define GS_CMD_USOSO            28
#define GS_CMD_USOST            29
#define GS_CMD_USOWR            30
#define GS_CMD_UUPSDA           31
#define GS_CMD_UUPSDD           32
#define GS_CMD_UUSOCL           33
#define GS_CMD_UUSOLI           34
#define GS_CMD_UUSORD           35
#define GS_CMD_UUSORF           36
#define GS_GET_CMD(cmdid) (&gs_commands[cmdid])

static const GSCmd gs_commands[] = {

    DEF_CMD("+CCID",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CCID),
    DEF_CMD("+CCLK",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CCLK),
    DEF_CMD("+CGATT",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CGATT),
    DEF_CMD("+CGED",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CGED),
    DEF_CMD("+CGSN",      GS_RES_NO, GS_CMD_NORMAL , GS_CMD_CGSN),
    DEF_CMD("+CIEV",      GS_RES_NO, GS_CMD_URC    , GS_CMD_CIEV),
    DEF_CMD("+CMEE",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CMEE),
    DEF_CMD("+CMER",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_CMER),
    DEF_CMD("+COPS",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_COPS),
    DEF_CMD("+CREG",      GS_RES_OK, GS_CMD_URC    , GS_CMD_CREG),
    DEF_CMD("+IPR",       GS_RES_OK, GS_CMD_NORMAL , GS_CMD_IPR),
    DEF_CMD("+UDCONF",    GS_RES_OK, GS_CMD_NORMAL , GS_CMD_UDCONF),
    DEF_CMD("+UDNSRN",    GS_RES_OK, GS_CMD_NORMAL , GS_CMD_UDNSRN),
    DEF_CMD("+UPSD",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_UPSD),
    DEF_CMD("+UPSDA",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_UPSDA),
    DEF_CMD("+UPSND",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_UPSND),
    DEF_CMD("+URAT",      GS_RES_OK, GS_CMD_NORMAL , GS_CMD_URAT),
    DEF_CMD("+USECMNG",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USECMNG),
    DEF_CMD("+USECPRF",   GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USECPRF),
    DEF_CMD("+USOCL",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOCL),
    DEF_CMD("+USOCO",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOCO),
    DEF_CMD("+USOCR",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOCR),
    DEF_CMD("+USOCTL",    GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOCTL),
    DEF_CMD("+USOGO",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOGO),
    DEF_CMD("+USOLI",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOLI),
    DEF_CMD("+USORD",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USORD),
    DEF_CMD("+USORF",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USORF),
    DEF_CMD("+USOSEC",    GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOSEC),
    DEF_CMD("+USOSO",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOSO),
    DEF_CMD("+USOST",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOST),
    DEF_CMD("+USOWR",     GS_RES_OK, GS_CMD_NORMAL , GS_CMD_USOWR),
    DEF_CMD("+UUPSDA",    GS_RES_NO, GS_CMD_URC    , GS_CMD_UUPSDA),
    DEF_CMD("+UUPSDD",    GS_RES_NO, GS_CMD_URC    , GS_CMD_UUPSDD),
    DEF_CMD("+UUSOCL",    GS_RES_NO, GS_CMD_URC    , GS_CMD_UUSOCL),
    DEF_CMD("+UUSOLI",    GS_RES_NO, GS_CMD_URC    , GS_CMD_UUSOLI),
    DEF_CMD("+UUSORD",    GS_RES_NO, GS_CMD_URC    , GS_CMD_UUSORD),
    DEF_CMD("+UUSORF",    GS_RES_NO, GS_CMD_URC    , GS_CMD_UUSORF),

};


