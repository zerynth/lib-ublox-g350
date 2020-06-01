/**
 * @file g350.c
 * @brief Driver for Sara G350 modules
 * @author Giacomo Baldi
 * @version 
 * @date 2017-08-20
 */



/** \mainpage Driver Architecture
 * 
 * This driver consists of:
 *
 * - a main thread (_gs_loop) that has exclusive access to the serial port in input
 * - a mechanism based on slots (GSlot) such that each thread calling into the driver must wait its turn to issue an AT command
 * - a list of socket structures (GSocket)
 *
 * The main thread reads one line at time and checks if it is a command response or not. In case it is a command response, it tries
 * to handle it based on the current slot. If the command response is a URC, it is handled by the corresponding functon (_gs_handle_urc),
 * otherwise if the line is not a command response, it is checked against "OK", "+CME ERROR", "ERROR" or ">" and action on the current slot
 * is taken.
 *
 * Once a slot is acquired for a particular command, the following can happen:
 *
 * - an OK is received and the thread owning the slot is signaled
 * - an error condition is received and the thread owning the slot is signaled
 * - the slot timeout is reached and the thread owning the slot is signaled
 * - a valid command response is detected and the gs.buffer is copied to the slot buffer for parsing
 * 
 * In all cases it is not possible for a slot to stall the main thread longer than timeout milliseconds. Obviously the thread owning
 * the slot must behave correctly:
 *
 * - acquire the slot
 * - send AT command
 * - wait for main thread signal
 * - parse the arguments from command if present/needed
 * - release the slot
 *
 * After slot release, the slot memory is no longer valid, unless allocated by the calling thread.
 *
 *
 */



#include "zerynth.h"
#include "g350.h"


//Enable/Disable debug printf
//#define UBLOX_SARA_G350_DEBUG 1

#ifdef UBLOX_SARA_G350_DEBUG
#define printf(...) vbl_printf_stdout(__VA_ARGS__)
#else
#define printf(...)
#endif



//STATIC VARIABLES

//the g350 driver status
static GStatus gs;
//the list of available sockets
static GSocket gs_sockets[MAX_SOCKS];
//the one and only slot available to threads
//to get the g350 driver attention
static GSSlot gslot;
//the list of GSM operators
static GSOp gsops[MAX_OPS];
//the number of GSM operators
static int gsopn=0;
//a reference to a Python exception to be returned on error (g350Exception)
static int32_t g350exc;


//Some declarations for socket handling
void _gs_socket_closing(int id);
void _gs_socket_pending(int id);


/**
 * @brief Initializes the data structures of gs
 */
void _gs_init(){
    int i;
    printf("Initializing GSM\n");
    for(i=0;i<MAX_SOCKS;i++){
        gs_sockets[i].lock = vosSemCreate(1);
        gs_sockets[i].rx = vosSemCreate(0);
    }
    memset(&gs,0,sizeof(GStatus));
    gs.slotlock = vosSemCreate(1);
    gs.sendlock = vosSemCreate(1);
    gs.slotdone = vosSemCreate(0);
    gs.secure_sock_id = -1;
    gs.initialized=0;
}

/**
 * @brief Clean up the data structures of gs
 */
void _gs_done(){
    int i;
    vhalSerialDone(gs.serial);
}


/**
 * @brief Begin the power up phase
 *
 * The function is tuned to G350 timings. It sets DTR and RTS to 0 to disable
 * hardware flow control. Sends an "AT" to the module until a response is received.
 *
 * @return 0 on success
 */
int _gs_poweron(){
    vhalPinSetMode(gs.poweron,PINMODE_OUTPUT_PUSHPULL);
    vhalPinWrite(gs.poweron,1);

    vhalPinSetMode(gs.reset,PINMODE_OUTPUT_PUSHPULL);
    vhalPinWrite(gs.reset,1);

    vhalSerialInit(gs.serial, 115200, SERIAL_CFG(SERIAL_PARITY_NONE,SERIAL_STOP_ONE, SERIAL_BITS_8,0,0), gs.rx, gs.tx);

    vhalPinSetMode(gs.dtr,PINMODE_OUTPUT_PUSHPULL);
    vhalPinWrite(gs.dtr,0);
    
    vhalPinSetMode(gs.rts,PINMODE_OUTPUT_PUSHPULL);
    vhalPinWrite(gs.rts,0);

    int retries = 0;

   for(;retries<20;retries++){
        printf("Power up sequence %i\n",retries); 
        //poweron
        vhalPinWrite(gs.poweron, 0);
        vosThSleep(TIME_U(100,MILLIS));  //minimum time 5 ms...let's be abundant :)
        vhalPinWrite(gs.poweron, 1);
        
        //reset
        if(retries==10){
            vhalPinWrite(gs.reset,0);
            vosThSleep(TIME_U(500,MILLIS)); //minimum time 50 ms
            vhalPinWrite(gs.reset,1);
        }
                  
        _gs_read(-1);
        vhalSerialWrite(gs.serial,"AT\r\n",4);
        vosThSleep(TIME_U(100,MILLIS));
        if(_gs_wait_for_ok(500)){
            retries=-1;
            break;
        }
    }

    if (retries<0) {
        printf("poweron ok\n");
        return 1;
    }
    printf("poweron ko\n");
    
    return 0;

}

/**
 * @brief Read lines from the module until a "OK" is received
 *
 * @param[in]   timeout     the number of milliseconds to wait for each line
 *
 * @return 0 on failure
 */
int _gs_wait_for_ok(int timeout){
    while(_gs_readline(timeout)>=0){
        if(_gs_check_ok()){
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Read a line from the module
 *
 * Lines are saved into gs.buffer and null terminated. The number of bytes read 
 * is saved in gs.buffer and returned. The timeout is implemented with a 50 milliseconds
 * polling strategy. TODO: change when the serial driver will support timeouts
 *
 * @param[in]   timeout     the number of milliseconds to wait for a line
 *
 * @return the number of bytes read or -1 on timeout
 */
int _gs_readline(int timeout){
    gs.bytes = 0;
    memset(gs.buffer,0,16);
    uint8_t *buf = gs.buffer;
    uint32_t tstart = vosMillis();
    while(gs.bytes<(MAX_BUF-1)){
        if(timeout>0) {
            if ((vosMillis()-tstart)>timeout) {
                *buf=0;
                return -1;
            }
            if(vhalSerialAvailable(gs.serial)>0){
                vhalSerialRead(gs.serial,buf,1);
            } else {
                vosThSleep(TIME_U(50,MILLIS));
                continue;
            }
        } else {
            vhalSerialRead(gs.serial,buf,1);
        }
        gs.bytes++;
//        printf("->%i\n",gs.bytes);
        if (*buf++=='\n') break;
    }
    //terminate for debugging!
    *buf=0;
    printf("rl: %s",gs.buffer);
    return gs.bytes;
}

/**
 * @brief 
 *
 * @param[in]   bytes   the number of bytes to read. If non positive, read the available ones.
 *
 *  Bytes are saved in gs.buffer and gs.bytes updated accordingly.
 *
 * @return the number of bytes read
 */
int _gs_read(int bytes){
    memset(gs.buffer,0,16);
    if (bytes<=0) bytes = vhalSerialAvailable(gs.serial);
    vhalSerialRead(gs.serial,gs.buffer,bytes);
    gs.bytes = bytes;
    gs.buffer[gs.bytes+1]=0;
    printf("rn: %s||\n",gs.buffer);
    return gs.bytes;
}

/**
 * @brief Checks if gs.buffer contains a valid "OK\r\n"
 *
 * @return 0 on failure
 */
int _gs_check_ok(){
    return memcmp(gs.buffer,"OK\r\n",4)==0 && gs.bytes>=4;
}

/**
 * @brief Checks if gs.buffer contains a valid error message
 *
 * Valid error messages may come from "+CME ERROR: " responses or
 * from "ERROR" responses (+USOxxx commands). Messages from "+CME" are
 * saved in gs.errmsg up to MAX_ERR_LEN.
 *
 * @return 0 on no error
 */
int _gs_check_error(){
    if (memcmp(gs.buffer,"+CME ERROR: ",12)==0 && gs.bytes>=12){
        int elen = MIN(gs.bytes-12,MAX_ERR_LEN);
        memcpy(gs.errmsg,gs.buffer+12,elen);
        gs.errlen = elen;
        return 1;
    } else if  (memcmp(gs.buffer,"ERROR",5)==0 && gs.bytes>=5) {
        gs.errlen=0;
        return 1;
    }
    return 0;
}


/**
 * @brief Checks if gs.buffer contains a known command response
 *
 * A binary search is performed on the known command, trying to match them
 * to gs.buffer.
 *
 * @return NULL on failure, a pointer to a GSCmd structure otherwise
 */
GSCmd* _gs_parse_command_response(){
    int e0=0,e1=KNOWN_COMMANDS-1,c=-1,r=0;
    GSCmd *cmd=NULL;
    while(e0<=e1){
        c=(e0+e1)/2;
        cmd = &gs_commands[c];
        //for this to work the first 16 bytes of gs.buffer must be zeroed at each read!
        //otherwise previous bytes can interfere
        r= memcmp(gs.buffer,cmd->body,cmd->len);
        if (r>0) e0=c+1;
        else if (r<0) e1=c-1;
        else break;
    }
    if(e0<=e1) {
        return cmd;
    }
    return NULL;
}

/**
 * @brief scans buf for bytes contained in pattern
 *
 * @param[in] buf       where to start the scan
 * @param[in] ebuf      where to end the scan
 * @param[in] pattern   characters to stop to
 *
 * @return a pointer to the location of one of the bytes in pattern or NULL if they cannot be found
 */
uint8_t* _gs_advance_to(uint8_t *buf,uint8_t *ebuf,uint8_t *pattern){
    uint8_t *pt;
    while(buf<ebuf){
        pt = pattern;
        while(*pt) {
            if(*buf==*pt) return buf;
            pt++;
        }
        buf++;
    }
    return NULL;
}

/**
 * @brief parse numbers in base 10 from a byte buffe 
 *
 * Does not check for number format correctness (0003 is ok) and
 * does not parse negative numbers. Skip spaces and \r \n without checking
 * if they break a number or not ("33 44" is parsed as 3344)
 *
 * @param[in]  buf     starting point
 * @param[in]  ebuf    ending point (not included)
 * @param[out] result  the positon reached after the parsing, NULL on failed parsing
 *
 * @return 
 */
uint8_t* _gs_parse_number(uint8_t *buf, uint8_t *ebuf, int32_t *result){
    int res = 0;
    while(buf<ebuf){
        if(*buf>='0' && *buf<='9'){
            res = res*10+(*buf-'0');
        } else if(*buf!=' ' && *buf!='\r' && *buf!='\n') return NULL; //allow spaces
        buf++;
    }
    if(result) *result = res;
    return buf;
}


/**
 * @brief parse the arguments of a command response
 *
 * Parse from buf to ebuf according to the fmt string. The fmt can contain
 * only "i" and "s". "i" needs a pointer to an int as the corresponsing variadic argument
 * while "s" needs two arguments: a uint8_t** to store the pointer to the string and an int32_t* to store
 * the string length. Parameters in buf are delimited by delimiters in this pattern: ",\r\n". Strings are not copied,
 * buf is modified in place by null terminating each parameter at the rightmost delimiter, and a pointer to the string parameter
 * is returned.
 *
 * Since buf is modified, arguments can be parsed only once. TODO: consider removing null termination since it is a feature
 * needed for debug only (printf)
 *
 * @param[in]  buf   starting point
 * @param[in]  ebuf  ending point (not included)
 * @param[in]  fmt   format string
 * @param[out] ...   variadic arguments (if NULL are parsed but not returned)
 *
 * @return the number of parameters parsed
 */
int _gs_parse_command_arguments(uint8_t *buf, uint8_t *ebuf, const char*fmt,...){
    va_list vl;
    va_start(vl, fmt);
    int32_t ret=0;
    int32_t *iparam;
    uint8_t **sparam;
    uint8_t *pms;
    uint8_t *pme=ebuf;
    int i;
    pms = buf;
    while(buf<ebuf){
        buf = _gs_advance_to(buf,ebuf,",\r\n");
        if(!buf) break;
        pme = buf-1;
        *buf=0;
        switch(*fmt) {
            case 0:
                goto exit;
            case 'i':
                iparam = va_arg(vl,int32_t*);
                if(iparam) *iparam=0;
                pms = _gs_parse_number(pms,pme+1,iparam);
                if(!pms) goto exit;
                ret++;
            break;
            case 's':
                sparam = va_arg(vl,uint8_t**);
                iparam = va_arg(vl,int32_t*);
                if(sparam) *sparam = pms;
                *buf=0; //end string
                if(iparam) *iparam = (pme-pms)+1; //store len
                ret++;
                break;
        }
        fmt++;
        pms=++buf;
    }

exit:
    va_end(vl);
    return ret;
}



/**
 * @brief send AT command to the module
 *
 * Send to serial an AT command identified by cmd_id (see g350.h).
 * Every byte following the command can be passed in fmt. If a byte in fmt equals "i"
 * an integer is expected as a variadic argument and "i" is expanded to the string representation
 * in base 10 of such integer. If a byte in fmt equals "s", two variadic arguments are expected: a uint8_t*
 * pointing to a byte buffer and an integer containing the number of bytes to copy in place of "s". Each other byte in fmt
 * is sent as is.
 *
 * @param[i] cmd_id  Macro identifying the command to send
 * @param[i] fmt     format string
 * @param[i] ...     variadic arguments
 */
void _gs_send_at(int cmd_id,const char *fmt,...){
    static uint8_t _strbuf[16];
    GSCmd *cmd = GS_GET_CMD(cmd_id);
    uint8_t *sparam;
    int32_t iparam;
    int32_t iparam_len;
    va_list vl;
    va_start(vl, fmt);

    vosSemWait(gs.sendlock);
    vhalSerialWrite(gs.serial,"AT",2);
    printf("->: AT");
    vhalSerialWrite(gs.serial,cmd->body,cmd->len);
    printf("%s",cmd->body);
    while(*fmt){
        switch(*fmt){
            case 'i':
                //number
                iparam = va_arg(vl, int32_t);
                iparam_len = modp_itoa10(iparam,_strbuf);
                vhalSerialWrite(gs.serial,_strbuf,iparam_len);
                _strbuf[iparam_len]=0;
                printf("%s",_strbuf);
                break;
            case 's':
                sparam = va_arg(vl, uint8_t *);
                iparam_len = va_arg(vl,int32_t *);
                vhalSerialWrite(gs.serial,sparam,iparam_len);
#if defined(UBLOX_SARA_G350_DEBUG)
                for(iparam=0;iparam<iparam_len;iparam++) printf("%c",sparam[iparam]);
#endif
                break;
            default:
                vhalSerialWrite(gs.serial,fmt,1);
                printf("%c",*fmt);
        }
        fmt++;
    }
    vhalSerialWrite(gs.serial,"\r",1);
    printf("\n");
    vosSemSignal(gs.sendlock);
    va_end(vl);
}

/**
 * @brief Configure basic parameters for startup
 *
 * Disables echo, set CMEE to 2, enable buffered urcs, set Hex mode for sockets and set CREG to 2.
 *
 * @return 0 on failure
 */
int _gs_config0(){
    //clean serial
    
    vhalSerialWrite(gs.serial,"ATE0\r\n",6);
    if(!_gs_wait_for_ok(500)) return 0;
    
    vhalSerialWrite(gs.serial,"AT+GMR\r\n",8);
    if(!_gs_wait_for_ok(500)) return 0;

    _gs_send_at(GS_CMD_CMEE,"=i",2);
    if(!_gs_wait_for_ok(500)) return 0;
    
    _gs_send_at(GS_CMD_CMER,"=i,i,i,i,i",2,0,0,2,1);
    if(!_gs_wait_for_ok(500)) return 0;
    
    _gs_send_at(GS_CMD_UDCONF,"=i,i",1,1); //enable HEX mode
    if(!_gs_wait_for_ok(1000)) return 0;

    _gs_send_at(GS_CMD_CREG,"=i",2);
    if(!_gs_wait_for_ok(500)) return 0;
    
    
    gs.initialized=1;
    return 1;
}

/**
 * @brief Check if a command response in gs.buffer is actually valid
 *
 * Validity is checked by making sure that the command respons is followed by ": "
 *
 * @param[in] cmd the command structure to check against
 *
 * @return the position of command arguments in gs.buffer, 0 on failure
 */
int _gs_valid_command_response(GSCmd *cmd){
    if (gs.buffer[cmd->len]==':' && gs.buffer[cmd->len+1]==' ' && gs.bytes>=cmd->len+2) {
        //valid
        return cmd->len+2;
    }
    return 0;
}

/**
 * @brief Handle received URC (unsolicited result code)
 *
 * Handled urcs are: CIEV, CREG, UUPSDA, UUSOCL, UUSORD, UUSORF.
 * In case of socket related URC, this function signals the socket
 * event semaphore to wake up threads suspended on a socket call.
 *
 * @param[in] cmd the GSCmd structure of the URC
 */
void _gs_handle_urc(GSCmd *cmd){
    int32_t p0,p1,p2,p3,nargs;
    uint8_t *s0,*s1,*s2,*s3;
    int p = _gs_valid_command_response(cmd);
    uint8_t *buf  = gs.buffer+p;
    uint8_t *ebuf = gs.buffer+gs.bytes;
    GSocket *sock;
    if(!p) return;

    switch(cmd->id){
        case GS_CMD_CIEV:
            if(_gs_parse_command_arguments(buf,ebuf,"ii",&p0,&p1)!=2) goto exit_err;
            switch(p0){
                case 2:
                    //rssi
                    gs.rssi = p1;
                    break;
                case 3:
                    //service
                    gs.registered = p1;
                    break;
                case 9:
                    gs.gprs = p1;
                    break;
                default:
                    printf("Unhandled +CIEV: %i %i\n",p0,p1);
            }
            break;
        case GS_CMD_CREG:
            nargs = _gs_parse_command_arguments(buf,ebuf,"issi",&p0,&s1,&p1,&s2,&p2,&p3);
            if(nargs<1) goto exit_err;
            switch(p0){
                case 3:
                    gs.registered = GS_REG_DENIED;
                    break;
                case 1:
                case 5:
                    gs.registered = (p0==1) ? GS_REG_OK:GS_REG_ROAMING;
                    if(nargs==4){
                        //got act
                        gs.gprs_mode = p3;
                    }
                    break;
                default:
                    gs.registered = GS_REG_NOT;
                    break;
            }            
            break;
        case GS_CMD_UUPSDA:
            nargs = _gs_parse_command_arguments(buf,ebuf,"i",&p0);
            if(nargs<1) goto exit_err;
            if(!p0) {
                //attached!
                gs.attached = 1;
            } else gs.attached = 0;
            break;
        case GS_CMD_UUSOCL:
            nargs = _gs_parse_command_arguments(buf,ebuf,"i",&p0);
            if(nargs<1) goto exit_err;
            _gs_socket_closing(p0);
            break;
        case GS_CMD_UUSORD:
        case GS_CMD_UUSORF:
            nargs = _gs_parse_command_arguments(buf,ebuf,"ii",&p0,&p1);
            if(nargs<2) goto exit_err;
            _gs_socket_pending(p0);
            break;
        default:
            printf("Unhandled URC %i\n",cmd->id);
    }

exit_ok:
    return;

exit_err:
    ;
    printf("Error parsing arguments for %i\n",cmd->id);
    return;
}

/**
 * @brief Wait for a slot to be available and acquires it
 *
 * A slot (or actually the slot, since in this implementation it is unique) is a structure holding information about the last issued command.
 * It also contains a buffer to hold the command response. Such buffer can be passed as an argument or (by passing NULL and a size) allocated by
 * the driver. In this case it will be deallocated on slot release. Acquiring a slot is a blocking operation and no other thread can access the serial port
 * until the slot is released.
 *
 * @param[i] cmd_id   the command identifier for the slot
 * @param[i] respbuf  a buffer sufficiently sized to hold the command response or NULL if such memory must be allocated by the driver
 * @param[i] max_size the size of respbuf. If 0 and respbuf is NULL, no memory is allocated. If positive and respbuf is NULL, memory is allocated by the driver
 * @param[i] timeout  the number of milliseconds before declaring the slot timed out
 * @param[i] nparams  the number of command response lines expected (0 or 1 in this implementation)
 *
 * @return a pointer to the acquired slot
 */
GSSlot *_gs_acquire_slot(int cmd_id, uint8_t *respbuf, int max_size, int timeout, int nparams){
    vosSemWait(gs.slotlock);
    gslot.cmd = GS_GET_CMD(cmd_id);
    gslot.stime = vosMillis();
    gslot.timeout = timeout;
    gslot.has_params = nparams;
    if(!respbuf){
        if(max_size){
            gslot.resp = gc_malloc(max_size);
            gslot.eresp = gslot.resp;
        } else {
            gslot.resp = gslot.eresp = NULL;
        }
        gslot.allocated = 1;
        gslot.max_size = max_size;
    } else {
        gslot.resp = gslot.eresp =respbuf;
        gslot.max_size = max_size;
        gslot.allocated = 0;
    }

    gs.slot = &gslot;
    return &gslot;
}

/**
 * @brief Wait until the main thread signal of slot completion
 */
void _gs_wait_for_slot(){
    vosSemWait(gs.slotdone);
}

/**
 * @brief Wait until the main thread signals the slot entering special mode (see +USOSECMNG)
 *
 * Special mode is requested by some commands. They do not return a response until some other lines of text
 * are sent after the AT command. textlen bytes are sent from text automatically before returning.
 * The calling thread must still wait for slot completion after this function returns.
 * This function can fail if the main thread does not signal spaecial mode after 10 seconds.
 *
 *
 * @param[in] text      the buffer to send
 * @param[in] textlen   the number of bytes to send
 *
 * @return 0 on success
 */
int _gs_wait_for_slot_mode(uint8_t *text, int32_t textlen){
    //can be polled!
    int cnt =0;
    printf("Waiting for mode\n");
    // vhalSerialWrite(gs.serial,">",1);
    while(gs.mode==GS_MODE_NORMAL &&cnt<100){ //after 10 seconds, timeout
        vosThSleep(TIME_U(100,MILLIS));
        cnt++;
    }

    if(gs.mode!=GS_MODE_PROMPT) return 1;
    printf("Slot wait mode\n");
    printf("-->%s\n",text);

    while(textlen>0){
        cnt = MIN(64,textlen);
        printf("Sending %i\n",cnt);
        cnt = vhalSerialWrite(gs.serial,text,cnt);
        printf("Sent %i\n",cnt);
        textlen-=cnt;
        text+=cnt;
        printf("Remaining %i\n",textlen);
    }
    gs.mode=GS_MODE_NORMAL; //back to normal mode

    return 0;
}

/**
 * @brief Release an acquired slot
 *
 * Deallocate slot memory if needed
 *
 * @param[in] slot the slot to release
 */
void _gs_release_slot(GSSlot *slot){
    if (slot->allocated && slot->resp) gc_free(slot->resp);
    memset(slot,0,sizeof(GSSlot));
    vosSemSignal(gs.slotlock);
}

/**
 * @brief Signal the current slot as ok
 */
void _gs_slot_ok(){
    printf("ok slot %s\n",gs.slot->cmd->body);
    gs.slot->err = 0;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Signal the current slot as error
 */
void _gs_slot_error(){
    printf("error slot %s\n",gs.slot->cmd->body);
    gs.slot->err = 2;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}

/**
 * @brief Signal the current slot as timed out
 */
void _gs_slot_timeout(){
    printf("timeout slot %s\n",gs.slot->cmd->body);
    gs.slot->err = GS_ERR_TIMEOUT;
    gs.slot = NULL;
    vosSemSignal(gs.slotdone);
}


/**
 * @brief Transfer the command response in gs.buffer to the slot memory
 *
 * @param[in] cmd the command to transfer
 */
void _gs_slot_params(GSCmd *cmd){
    //copy params to slot
    if (!gs.slot->resp) return;
    if(cmd->response_type == GS_RES_NO) {
        int csize = (gs.slot->max_size<gs.bytes) ? gs.slot->max_size:gs.bytes;
        memcpy(gs.slot->resp,gs.buffer,csize);
        gs.slot->eresp = gs.slot->resp+csize;
    } else {
        if (!_gs_valid_command_response(cmd)) return;
        int psize = gs.bytes-cmd->len-2;
        int csize = (gs.slot->max_size<psize) ? gs.slot->max_size:psize;
        memcpy(gs.slot->resp,gs.buffer+cmd->len+2,csize);
        gs.slot->eresp = gs.slot->resp+csize;
    }
    gs.slot->params++;
}

/**
 * @brief Main thread loop
 *
 * Exit when the driver is deinitialized
 *
 * @param[i] args thread arguments
 */
void _gs_loop(void *args){
    (void)args;
    GSCmd *cmd;
    printf("_gs_loop started\n");
    while (gs.initialized){
        // printf("looping\n");
        if(gs.mode != GS_MODE_PROMPT){
            if(_gs_readline(100)<=3){
                if(gs.bytes>=1 && gs.buffer[0]=='>' && gs.slot && gs.slot->cmd->id == GS_CMD_USECMNG){
                    //only enter in prompt mode if the current slot is for USECMNG to avoid locks
                    printf("GOT PROMPT!\n");
                    gs.mode = GS_MODE_PROMPT;
                    continue;
                }
                //no line
                if (gs.slot) {
                    if (gs.slot->timeout && (vosMillis()-gs.slot->stime)>gs.slot->timeout){
                        //slot timed out
                        printf("slot timeout\n");
                        _gs_slot_timeout();
                    }
                }
                continue;
            }
            cmd = _gs_parse_command_response();
            if (gs.slot) {
                //we have a slot
                if (cmd) {
                    //we parsed a command
                    if(cmd == gs.slot->cmd) {
                        //we parsed the response to slot
                        if (gs.slot->has_params){
                            printf("filling slot params for %s\n",cmd->body);
                            _gs_slot_params(cmd);
                        } else {
                            printf("Unexpected params for slot\n");
                        }
                    } else if (cmd->urc) {
                        //we parsed a urc
                        printf("Handling urc %s in a slot\n",cmd->body);
                        _gs_handle_urc(cmd);
                    }
                } else {
                    //we don't have a command
                    if (_gs_check_ok()){
                        //we got an OK...is it for the current slot?
                        if (gs.slot->has_params == gs.slot->params){
                            _gs_slot_ok();
                        } else {
                            printf("Unexpected OK\n");
                        }
                    } else if (_gs_check_error()){
                        _gs_slot_error();
                    } else if(gs.slot->cmd->response_type == GS_RES_NO) {
                        //the command behaves differently
                        printf("filling slot params for GS_RES_NO\n");
                        _gs_slot_params(gs.slot->cmd);
                    } else{
                        printf("Unexpected line\n");
                    }
                }
            } else {
                // we have no slot
                if  (cmd) {
                    //we have a command
                    if(cmd->urc) {
                        printf("Handling urc %s out of slot\n",cmd->body);
                        _gs_handle_urc(cmd);
                    } else {
                        printf("Don't know what to do with %s\n",cmd->body);
                    }
                } else {
                    // we have no command
                    printf("Unknown line out of slot\n");
                }
            }
        } else {
            //// PROMPT MODE 
            //Prompt mode is used for USECMNG (implemented) and DWNFILE (not implemented)
            //If needed, logic for prompt mode goes here
            int ss;
            for(ss=0;ss<40;ss++){
                //avoid locking, max time spendable in prompt mode = 20s
                vosThSleep(TIME_U(500,MILLIS));
                if (gs.mode!=GS_MODE_PROMPT) break;
            }
            gs.mode = GS_MODE_NORMAL;

        }
    }

}


///////// CNATIVES
// The following functions are callable from Python.
// Functions starting with "_" are utility functions called by CNatives



/**
 * @brief _g350_init calls _gs_init, _gs_poweron and _gs_config0
 *
 * As last parameter, requires an integer saved to global g350exc, representing the name assigned to g350Exception
 * so that it can be raised by returning g350exc. If modules initialization is successful, starts the main thread
 *
 */
C_NATIVE(_g350_init){
    NATIVE_UNWARN();
    int32_t serial;
    int32_t rx;
    int32_t tx;
    int32_t rts;
    int32_t dtr;
    int32_t poweron;
    int32_t reset;
    int32_t err = ERR_OK;  
    int32_t exc;
    
        
    if (parse_py_args("iiiiii", nargs, args, &serial, &dtr, &rts, &poweron, &reset, &exc) != 6)
        return ERR_TYPE_EXC;

    g350exc = exc;

    *res = MAKE_NONE();


    RELEASE_GIL();
    _gs_init();
    gs.serial = serial&0xff;
    gs.rx = _vm_serial_pins[gs.serial].rxpin;
    gs.tx = _vm_serial_pins[gs.serial].txpin;
    gs.dtr = dtr;
    gs.rts = rts;
    gs.poweron = poweron;
    gs.reset = reset;
    if(!_gs_poweron()) {
        err = ERR_HARDWARE_INITIALIZATION_ERROR;
    } 
    else {
        if(!_gs_config0()) err = ERR_HARDWARE_INITIALIZATION_ERROR;
    }
    ACQUIRE_GIL();

    if(err==ERR_OK){
        //let's start modem thread
        printf("Starting modem thread with size %i\n",VM_DEFAULT_THREAD_SIZE);
        gs.thread = vosThCreate(VM_DEFAULT_THREAD_SIZE,VOS_PRIO_NORMAL,_gs_loop,NULL,NULL);
        vosThResume(gs.thread);
    }

    return err;
}


/**
 * @brief Generalize sending AT commands for +UPSD (setting the packed switched data profile)
 *
 * @param[in] tag    tag parameter for +UPSD 
 * @param[in] param  string parameter for +UPSD
 * @param[in] len    length of param
 *
 * @return 0 on failure
 */
int _g350_configure_psd(int tag, uint8_t *param, int len){
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_UPSD,NULL,0,GS_TIMEOUT,0);
    if (param) _gs_send_at(GS_CMD_UPSD,"=i,i,\"s\"",GS_PROFILE,tag,param,len);
    else  _gs_send_at(GS_CMD_UPSD,"=i,i,i",GS_PROFILE,tag,len);
    _gs_wait_for_slot();
    int res = !slot->err;
    _gs_release_slot(slot);
    return res;
}

/**
 * @brief Generalize sending AT commands for +UPSDA (activate psd profile)
 *
 * @param[in] tag   tag parameter for +UPSDA
 *
 * @return 0 on failure
 */
int _g350_control_psd(int tag){
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_UPSDA,NULL,0,GS_TIMEOUT*60*3,0);
    _gs_send_at(GS_CMD_UPSDA,"=i,i",GS_PROFILE,tag);
    _gs_wait_for_slot();
    int res = !slot->err;
    _gs_release_slot(slot);
    return res;
}


/**
 * @brief Generalize sending AT commands for +UPSND (query psd data)
 *
 * @param[in]  query     query parameter for +UPSND
 * @param[out] param     stores the pointer to the string result of +UPSND (ignored if NULL)
 * @param[out] param_len stores the length of returned param
 *
 * @return 0 on failure
 */
int _g350_query_psd(int query, uint8_t **param, uint32_t *param_len){
    GSSlot *slot;
    int p0,p1,p2;
    slot = _gs_acquire_slot(GS_CMD_UPSND,NULL,32,GS_TIMEOUT*5,1);
    _gs_send_at(GS_CMD_UPSND,"=i,i",GS_PROFILE,query);
    _gs_wait_for_slot();
    if (param) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"iis",&p0,&p1,param,param_len)!=3) {
            _gs_release_slot(slot);
            return 0;
        }
        p1=1;
    } else {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"iii",&p0,&p1,&p2)!=3) {
            _gs_release_slot(slot);
            return 0;
        }
    }
    _gs_release_slot(slot);
    return p1;
}


int _g350_check_network(){
    GSSlot *slot;
    int p0,p1,p2;
    slot = _gs_acquire_slot(GS_CMD_CREG,NULL,64,GS_TIMEOUT*5,1);
    _gs_send_at(GS_CMD_CREG,"?");
    _gs_wait_for_slot();

    if(_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&p0,&p1)!=2) {
        _gs_release_slot(slot);
        return 0;
    }
    _gs_release_slot(slot);
    if(p1==1 || p1==5) gs.registered = (p1==1) ? GS_REG_OK:GS_REG_ROAMING;
    return p1;
}

C_NATIVE(_new_check_network){
    NATIVE_UNWARN();
    printf("_new_check_network 1\n");
    
    GSSlot *slot;
    int p0,p1,p2;
    PTuple *tpl = ptuple_new(2,NULL);
    
    PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW(-1));
    PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(-1));
    printf("_new_check_network 2\n");
    slot = _gs_acquire_slot(GS_CMD_CREG,NULL,64,GS_TIMEOUT*5,1);
    _gs_send_at(GS_CMD_CREG,"?");
    _gs_wait_for_slot();
    printf("_new_check_network 3\n");
    if(_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&p0,&p1)!=2) {
        _gs_release_slot(slot);
        *res = tpl;
        return ERR_OK;
    }
    _gs_release_slot(slot);
    printf("_new_check_network 4\n");
    PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW(p0));
    PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(p1));
    *res = tpl;
    return ERR_OK;
}

/**
 * @brief _g350_detach removes the link with the APN while keeping connected to the GSM network
 *
 * 
 */
C_NATIVE(_g350_detach){
    NATIVE_UNWARN();
    return ERR_OK;
}



/**
 * @brief _g350_attach tries to link to the given APN
 *
 * This function can block for a very long time (up to 2 minutes) due to long timeout of used AT commands
 *
 *
 */
C_NATIVE(_g350_attach){
    NATIVE_UNWARN();
    uint8_t *apn;
    uint32_t apn_len;
    uint8_t *user;
    uint32_t user_len;
    uint8_t *password;
    uint32_t password_len;
    uint32_t authmode;
    int32_t timeout;
    int32_t wtimeout;
    int32_t err=ERR_OK;
    
    int i;


    if(parse_py_args("sssii",nargs,args,&apn,&apn_len,&user,&user_len,&password,&password_len,&authmode,&wtimeout)!=5) return ERR_TYPE_EXC;
    
    *res = MAKE_NONE();
    GSSlot *slot = NULL;
    RELEASE_GIL();
    
    //Attach to GPRS
    slot = _gs_acquire_slot(GS_CMD_CGATT,NULL,0,GS_TIMEOUT*60*3,0);
    _gs_send_at(GS_CMD_CGATT,"=i",1);
    _gs_wait_for_slot();
    if(slot->err) {
        if (slot->err==GS_ERR_TIMEOUT) err = ERR_TIMEOUT_EXC;
        else err = g350exc;
        _gs_release_slot(slot);
        goto exit;
    }
    _gs_release_slot(slot);
    //wait until timeut or GPRS attached (by urc +CREG or +CIEV)
    timeout=wtimeout;
    while(timeout>0){
        _g350_check_network();
        if (gs.registered==GS_REG_OK || gs.registered==GS_REG_ROAMING) break;
        vosThSleep(TIME_U(100,MILLIS));
        timeout-=100;
    }
    if(timeout<0) {
        err = ERR_TIMEOUT_EXC;
        goto exit;
    }
    
    //deactivate PSD
    _g350_control_psd(4);

    //Get profile status (if already linked, ignore the following configuration)
    // i = _g350_query_psd(8,NULL,NULL);
    // if(i==1) {
        // goto exit;
    // }
    //configure PSD: give apn first, then username, password and authmode
    err = g350exc;
    if(!_g350_configure_psd(1,apn,apn_len)) goto exit;
    if(user_len){
        if(!_g350_configure_psd(2,user,user_len)) goto exit;
    }
    if (password_len) {
        if(!_g350_configure_psd(3,password,password_len)) goto exit;
    }
    if(!_g350_configure_psd(6,NULL,authmode)) goto exit; 

    
    //activate PSD
    gs.attached = 0;
    if(!_g350_control_psd(3)) goto exit; 

    //wait for attached (set by +UUPSDA or queried by +UPSND)
    timeout=wtimeout;
    while(timeout>0){
        if(gs.attached) break;
        vosThSleep(TIME_U(1000,MILLIS));
        timeout-=1000;
        if (_g350_query_psd(8,NULL,NULL)){
            gs.attached = 1;
            break;
        }
    }
    if(timeout<0) err = ERR_TIMEOUT_EXC;
    else err = ERR_OK;

    exit:
    ACQUIRE_GIL();
    *res = MAKE_NONE();
    return err;
}

/**
 * @brief Retrieve the list of operators with +COPS test command
 *
 * If successful, stores the retrieved operators with their parameters
 * in the global operator list (capped at MAX_OPS) and set their number (gsopn) accordingly
 *
 * @return 0 on success
 */
int _gs_list_operators(){
    GSSlot *slot;
    int err;
    slot = _gs_acquire_slot(GS_CMD_COPS,NULL,MAX_CMD,GS_TIMEOUT*60,1);
    _gs_send_at(GS_CMD_COPS,"=?");
    _gs_wait_for_slot();
    if (slot->err) {
        err = slot->err;
        _gs_release_slot(slot);
        return err;    
    }
    uint8_t *buf = slot->resp;
    uint8_t nops =0, nres, nt=0;
    while(buf<slot->eresp){
        if (!(*buf=='(' && *(buf+3)=='"')) break; //not a good record
        buf++; //skip (
        gsops[nops].type=*buf-'0';
        buf++; buf++; buf++; //skip ,"
        nt=0;
        while(*buf!='"'){
            gsops[nops].fmt_long[nt++]=*buf++;
        }
        gsops[nops].fmtl_l=nt;
        buf++; buf++; buf++; //skip ","
        nt=0;
        while(*buf!='"'){
            gsops[nops].fmt_short[nt++]=*buf++;
        }
        gsops[nops].fmts_l=nt;
        buf++; buf++; buf++; //skip ","
        nt=0;
        while(*buf!='"'){
            gsops[nops].fmt_code[nt++]=*buf++;
        }
        gsops[nops].fmtc_l=nt;
        buf++; buf++; buf++; //skip "),
        nops++;
        if (nops==MAX_OPS) break;
    }
    gsopn = nops;
    _gs_release_slot(slot);
    return 0;
}

/**
 * @brief Try to set the current operator
 *
 * @param[in] opname    long operator name as returned by +COPS
 * @param[in] oplen     length of param
 *
 * @return 0 on success
 */
int _gs_set_operator(uint8_t *opname, uint32_t oplen){
    GSSlot *slot;
    int err;
    slot = _gs_acquire_slot(GS_CMD_COPS,NULL,NULL,GS_TIMEOUT*60,0);
    _gs_send_at(GS_CMD_COPS,"=1,0,\"s\"",opname,oplen);
    _gs_wait_for_slot();
    if (slot->err) {
        err = slot->err;
        _gs_release_slot(slot);
        return err;
    }    
    _gs_release_slot(slot);
    return 0;
}

/**
 * @brief _g350_operators retrieve the operator list and converts it to a tuple
 *
 * 
 */
C_NATIVE(_g350_operators){
    NATIVE_UNWARN();
    int i;

    RELEASE_GIL();
    i = _gs_list_operators();
    ACQUIRE_GIL();
    if (i){
        *res = MAKE_NONE();
        return ERR_OK;
    }
    PTuple *tpl = ptuple_new(gsopn,NULL);
    for(i=0;i<gsopn;i++){
        PTuple *tpi = ptuple_new(4,NULL);
        PTUPLE_SET_ITEM(tpi,0,PSMALLINT_NEW(gsops[i].type));
        PTUPLE_SET_ITEM(tpi,1,pstring_new(gsops[i].fmtl_l,gsops[i].fmt_long));
        PTUPLE_SET_ITEM(tpi,2,pstring_new(gsops[i].fmts_l,gsops[i].fmt_short));
        PTUPLE_SET_ITEM(tpi,3,pstring_new(gsops[i].fmtc_l,gsops[i].fmt_code));
        PTUPLE_SET_ITEM(tpl,i,tpi);
    }

    *res = tpl;
    return ERR_OK;
}


/**
 * @brief _g360_set_operator try to set the current operator given its name
 *
 * 
 */
C_NATIVE(_g350_set_operator){
    NATIVE_UNWARN();
    int i;
    uint8_t *opname;
    uint32_t oplen;

    if(parse_py_args("s",nargs,args,&opname,&oplen)!=1) return ERR_TYPE_EXC;

    RELEASE_GIL();
    i = _gs_set_operator(opname,oplen);
    ACQUIRE_GIL();
    *res = MAKE_NONE();
    if (i==GS_TIMEOUT){
        return ERR_TIMEOUT_EXC;
    } else {
        return g350exc;
    }
    return ERR_OK;
}


/**
 * @brief _g350_last_error retrieve the last error sring generated by +CME ERROR
 *
 * 
 */
C_NATIVE(_g350_last_error){
    NATIVE_UNWARN();
    PString *ps = pstring_new(gs.errlen,gs.errmsg);
    *res = ps;
    return ERR_OK;
}

int _g350_get_rtc(uint8_t* time)
{
    GSSlot* slot;
    uint8_t* s0;
    int l0;
    int res;
    slot = _gs_acquire_slot(GS_CMD_CCLK, NULL, 32, GS_TIMEOUT, 1);
    _gs_send_at(GS_CMD_CCLK, "?");
    _gs_wait_for_slot();
    res = !slot->err;
    if (res) {
        if (_gs_parse_command_arguments(slot->resp, slot->eresp, "s", &s0, &l0) != 1) {
            res = 0;
        } else {
            memcpy(time, s0 + 1, 20);
        }
    }
    _gs_release_slot(slot);

    return res;
}

/**
 * @brief _g350_get_clock reads the real-time clock of the MT by means of +CCLK
 *
 *
 */
C_NATIVE(_g350_rtc){
    C_NATIVE_UNWARN();
    int err = ERR_OK;
    uint8_t time[20];
    *res = MAKE_NONE();
    memset(time,0,20);
    RELEASE_GIL();
    if(!_g350_get_rtc(time)) err=ERR_RUNTIME_EXC;
    ACQUIRE_GIL();
    if (err==ERR_OK) {
        PTuple* tpl = ptuple_new(7,NULL);
        int yy,MM,dd,hh,mm,ss,tz;
        yy = 2000+((time[0]-'0')*10+(time[1]-'0'));
        MM = (time[3]-'0')*10+(time[4]-'0');
        dd = (time[6]-'0')*10+(time[7]-'0');
        hh = (time[9]-'0')*10+(time[10]-'0');
        mm = (time[12]-'0')*10+(time[13]-'0');
        ss = (time[15]-'0')*10+(time[16]-'0');
        tz = ((time[18]-'0')*10+(time[19]-'0'))*15*((time[17]=='-')?-1:1);
        PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW(yy));
        PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(MM));
        PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(dd));
        PTUPLE_SET_ITEM(tpl,3,PSMALLINT_NEW(hh));
        PTUPLE_SET_ITEM(tpl,4,PSMALLINT_NEW(mm));
        PTUPLE_SET_ITEM(tpl,5,PSMALLINT_NEW(ss));
        PTUPLE_SET_ITEM(tpl,6,PSMALLINT_NEW(tz));
        *res = tpl;
    }
    return err;
}

/**
 * @brief _g350_rssi return the signal strength as reported by +CIEV urc
 *
 * 
 */
C_NATIVE(_g350_rssi){
    NATIVE_UNWARN();
    int32_t rssi=-105+12*gs.rssi;

    *res = PSMALLINT_NEW(rssi);
    return ERR_OK;
}



/**
 * @brief strings for network types
 */
static const uint8_t *_urats[] = {
    "GSM",
    "UMTS",
    "LTE"
};



/**
 * @brief _g450_network_info retrieves network information through +URAT and *CGED
 *
 *
 */
C_NATIVE(_g350_network_info){
    NATIVE_UNWARN();
    int p0,l0,l1,l2,l3,l4;
    uint8_t *s0,*s1,*s2,*s3,*s4,*st,*se;
    PString *urat;
    PString *tstr=NULL;
    PTuple *tpl = ptuple_new(8,NULL);

    //RAT  : URAT
    //CELL : UCELLINFO
    GSSlot *slot; 
    RELEASE_GIL();

    // GET URAT
    slot = _gs_acquire_slot(GS_CMD_URAT,NULL,32,GS_TIMEOUT*10,1);
    _gs_send_at(GS_CMD_URAT,"?");
    _gs_wait_for_slot();
    p0 = 0;
    //+URAT is not always supported in G3 family, default to GSM in case of error
    if (!slot->err){
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"i",&p0)==1) {
            if (p0==2) p0=1;
            else if(p0>=3) p0=2;
        }
    }
    urat = pstring_new( (p0==1)?4:3,_urats[p0]);
    PTUPLE_SET_ITEM(tpl,0,urat);
    _gs_release_slot(slot);

    //GET CELLINFO
    slot = _gs_acquire_slot(GS_CMD_CGED,NULL,512,GS_TIMEOUT*10,1);
    _gs_send_at(GS_CMD_CGED,"=i",3);
    _gs_wait_for_slot();
    if (!slot->err){
        //only 3G and 2G supported!
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"sssss",&s0,&l0,&s1,&l1,&s2,&l2,&s3,&l3,&s4,&l4)==5){
            //MCC
            se = s0+l0;
            st = s0;
            st = _gs_advance_to(st,se,":");
            if (st) {
                if(st=_gs_parse_number(st+1,se,&p0)){
                    PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(p0));
                }
            }
            if(!st){
                PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(-1));
            }
            //MNC
            se = s1+l1;
            st = s1;
            st = _gs_advance_to(st,se,":");
            if (st) {
                if(st=_gs_parse_number(st+1,se,&p0)){
                    PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(p0));
                }
            }
            if(!st){
                PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(-1));
            }
            //BSIC
            se = s4+l4;
            st = s4;
            st = _gs_advance_to(st,se,":");
            if (st) {
                tstr = pstring_new(se-st,st+1);
                PTUPLE_SET_ITEM(tpl,3,tstr);
            } else {
                PTUPLE_SET_ITEM(tpl,3,pstring_new(0,NULL));
            }
            //LAC
            se = s2+l2;
            st = s2;
            st = _gs_advance_to(st,se,":");
            if (st) {
                tstr = pstring_new(se-st,st+1);
                PTUPLE_SET_ITEM(tpl,4,tstr);
            } else {
                PTUPLE_SET_ITEM(tpl,4,pstring_new(0,NULL));
            }
            //CI
            se = s3+l3;
            st = s3;
            st = _gs_advance_to(st,se,":");
            if (st) {
                tstr = pstring_new(se-st,st+1);
                PTUPLE_SET_ITEM(tpl,5,tstr);
            } else {
                PTUPLE_SET_ITEM(tpl,5,pstring_new(0,NULL));
            }
            
        }
    }
    if(!PTUPLE_ITEM(tpl,1)){
        //empty result
        tstr = pstring_new(0,NULL);
        PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(-1));
        PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(-1));
        PTUPLE_SET_ITEM(tpl,3,tstr);
        PTUPLE_SET_ITEM(tpl,4,tstr);
        PTUPLE_SET_ITEM(tpl,5,tstr);
    }
    _gs_release_slot(slot);

    //registered to network
    PTUPLE_SET_ITEM(tpl,6,gs.registered ? PBOOL_TRUE():PBOOL_FALSE());
    //attached to APN
    PTUPLE_SET_ITEM(tpl,7,gs.attached ? PBOOL_TRUE():PBOOL_FALSE());
    
    ACQUIRE_GIL();
    *res = tpl;
    return ERR_OK;
}

/**
 * @brief _g350_mobile_info retrieves info on IMEI and SIM card by means of +CGSN and *CCID
 *
 * 
 */
C_NATIVE(_g350_mobile_info){
    NATIVE_UNWARN();
    
    PTuple *tpl = ptuple_new(2,NULL);

    //IMEI : CGN    
    //SIM : CCID
    GSSlot *slot; 
    RELEASE_GIL();

    //GET IMEI
    slot = _gs_acquire_slot(GS_CMD_CGSN,NULL,64,GS_TIMEOUT*10,1);
    _gs_send_at(GS_CMD_CGSN,"");
    _gs_wait_for_slot();
    if(!slot->err){
        uint8_t *se = _gs_advance_to(slot->resp,slot->eresp,"\r\n");
        if (se) {
            PTUPLE_SET_ITEM(tpl,0,pstring_new(se-slot->resp,slot->resp));
        }
    }
    _gs_release_slot(slot);

    //GET SIM SN
    slot = _gs_acquire_slot(GS_CMD_CCID,NULL,64,GS_TIMEOUT*10,1);
    _gs_send_at(GS_CMD_CCID,"");
    _gs_wait_for_slot();
    if(!slot->err){
        uint8_t *s0;
        uint32_t l0;
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"s",&s0,&l0)==1){
            PTUPLE_SET_ITEM(tpl,1,pstring_new(l0,s0));
        }
    }
    _gs_release_slot(slot);
    
    if (!PTUPLE_ITEM(tpl,0)){
        PTUPLE_SET_ITEM(tpl,0,pstring_new(0,NULL));
    }
    if (!PTUPLE_ITEM(tpl,1)){
        PTUPLE_SET_ITEM(tpl,1,pstring_new(0,NULL));
    }

    ACQUIRE_GIL();
    *res =tpl;
    return ERR_OK;
}

/**
 * @brief _g350_link_info retrieves ip and dns by means of +UPSND
 *
 *
 */
C_NATIVE(_g350_link_info){
    NATIVE_UNWARN();
    PString *ips;
    PString *dns;
    uint8_t *addr;
    uint32_t addrlen; 

    RELEASE_GIL();
    
    if(_g350_query_psd(0,&addr,&addrlen)){
        ips = pstring_new(addrlen-2,addr+1);
    } else {
        ips = pstring_new(0,NULL);
    }

    if(_g350_query_psd(1,&addr,&addrlen)){
        dns = pstring_new(addrlen-2,addr+1);
    } else {
        dns = pstring_new(0,NULL);
    }

    ACQUIRE_GIL();
    PTuple *tpl = ptuple_new(2,NULL);
    PTUPLE_SET_ITEM(tpl,0,ips);
    PTUPLE_SET_ITEM(tpl,1,dns);
    *res = tpl;
    return ERR_OK;
}


/////////////////////SOCKET HANDLING
//
// The following functions implemented BSD compatible sockets on top of AT commands.
// CNatives and utilities functions follow the convention above


/**
 * \mainpage Socket Management
 * 
 *  GSocket structure contains two semaphores, one to gain exclusive access to the structure (socklock) the other
 *  to signal event to threads suspended on a socket receive. Since sockets can be closed remotely, GSocket also has a flag (to_be_closed) 
 *  indicating such event.
 *
 *  The id of a socket (index in the socket list) is assigned by the +USOCR command. If a previously created GSocket with the same id
 *  returned by +USOCR has not been properly closed, the creation of the corresponding new GSocket fails until correct closing. This prevents memory leaks
 *  and the possibility of having in Python two sockets instances with the same id (one valid and one invalid).
 *
 *  The event about pending bytes in the receive queue is signaled by the module with one or more URCs. 
 *  The URC handling routine signal the appropriate socket on the rx semaphore.
 *  There is no check about the status of the socket (this would have involved a more complicated coordination between he main thread
 *  and sockets). 
 *
 *  CNatives that implements the various form of recv suspend themselves on the rx semaphore when the module AT command 
 *  (+USORD or +USORF) return 0 available bytes. However, since the urc handling routine may signal pending bytes repeatedly, it can happen that
 *  even at 0 available bytes, rx semaphore won't suspend. This is not a big issue, since the additional (and unneeded) AT command executions are quite fast.
 *
 *
 */




#define DRV_SOCK_DGRAM 1
#define DRV_SOCK_STREAM 0
#define DRV_AF_INET 0


/**
 * @brief creates a new socket with id and proto
 *
 * Sockets are numbered from 0 to MAX_SOCKS by the g350 module on creation, therefore
 * creating a GSocket means initializing the structure with default values.
 *
 * @param[in] id    the id from 0 to MAX_SOCKS
 * @param[in] proto the proto type (6=TCP, 17=UDP)
 *
 * @return a pointer to the created socket or NULL on error
 */
GSocket *_gs_socket_new(int id, int proto){
    GSocket *sock, *res=NULL;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);

    if(!sock->acquired) {
        sock->acquired      = 1;
        sock->to_be_closed  = 0;
        sock->timeout       = 0;
        sock->proto         = proto;
        res = sock;
    }
    
    vosSemSignal(sock->lock);
    return res;
}

/**
 * @brief retrieve the socket with a specific id if it exists
 *
 * @param[in] id    the socket id
 *
 * @return the socket or NULL on failure
 */
GSocket *_gs_socket_get(int id){
    GSocket *sock, *res=NULL;

    sock = &gs_sockets[id];
    vosSemWait(sock->lock);

    if(sock->acquired)  res = sock;
    
    vosSemSignal(sock->lock);
    return res;
}

/**
 * @brief 
 *
 * @param id
 */
void _gs_socket_close(int id){
    GSocket *sock;
    sock = &gs_sockets[id];

    vosSemWait(sock->lock);
    vosSemSignal(sock->rx);
    if (gs.secure_sock_id==id) gs.secure_sock_id=-1; //free tls
    sock->acquired = 0;
    sock->to_be_closed = 1;
    vosSemSignal(sock->lock);
}

void _gs_socket_closing(int id){
    GSocket *sock;
    sock = &gs_sockets[id];

    vosSemWait(sock->lock);
    vosSemSignal(sock->rx);
    sock->to_be_closed=1;
    if (gs.secure_sock_id==id) gs.secure_sock_id=-1; //free tls
    vosSemSignal(sock->lock);
}

void _gs_socket_pending(int id){
    GSocket *sock;
    sock = &gs_sockets[id];

    vosSemWait(sock->lock);
    vosSemSignal(sock->rx);
    vosSemSignal(sock->lock);
}



int _gs_socket_wait_rx(GSocket *sock,int timeout){
    return vosSemWaitTimeout(sock->rx, (timeout<0) ? VTIME_INFINITE:TIME_U(timeout,MILLIS));
}


uint8_t* _gs_socket_hex_to_bin(uint8_t *hex, uint8_t *buf, int bytes){
    uint8_t c;
    int i;
    while(bytes-->0){
        c = 0;
        for(i=0;i<2;i++,hex++){
            if(*hex>='0' && *hex<='9') c=c*16+(*hex-'0');
            else if(*hex>='A' && *hex<='F') c=c*16+10+(*hex-'A');
            else if(*hex>='a' && *hex<='f') c=c*16+10+(*hex-'a');
        }
        *buf++=c;
    }
    return buf;
}

uint8_t* _gs_socket_bin_to_hex(uint8_t *buf, uint8_t *hex, int bytes){
    uint8_t h,l;
    int i;
    while(bytes-->0){
        h = (*buf&0xf0)>>4;
        l = *buf&0x0f;
        if(h>=10) *hex='A'+(h-10);
        else *hex='0'+h;
        hex++;
        if(l>=10) *hex='A'+(l-10);
        else *hex='0'+l;
        hex++;
        buf++;
    }
    return buf;
}

int _gs_socket_addr(NetAddress *addr, uint8_t *saddr) {
    uint8_t *buf = saddr;
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 0),buf);
    *buf++='.';
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 1),buf);
    *buf++='.';
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 2),buf);
    *buf++='.';
    buf+=modp_itoa10(OAL_IP_AT(addr->ip, 3),buf);
    return buf-saddr; 
}

int _gs_socket_error( int sock){
    int p0=-1,p1,p2;
    GSSlot *slot;
    slot = _gs_acquire_slot(GS_CMD_USOCTL,NULL,16,GS_TIMEOUT,1);
    _gs_send_at(GS_CMD_USOCTL,"=i,i",sock,1);
    _gs_wait_for_slot();
    if (!slot->err) {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"iii",&p0,&p1,&p2)!=3) {
            p0=-1;
        } else {
            p0=p2;
        }
    }
    _gs_release_slot(slot);
    return p0;
}

int _g350_usocr(int proto){
    int err;
    GSSlot *slot = _gs_acquire_slot(GS_CMD_USOCR,NULL,32,GS_TIMEOUT*2,1);
    _gs_send_at(GS_CMD_USOCR,"=i",proto);
    _gs_wait_for_slot();
    if(slot->err) {
        err = -1;
    } else {
        int p0;
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"i",&p0)==1){
            if(!_gs_socket_new(p0,proto)){
                //a previous socket bound to the same id has not been closed properly!!
                //close the just created socket and return error
                _gs_release_slot(slot);
                slot = _gs_acquire_slot(GS_CMD_USOCL,NULL,0,GS_TIMEOUT*15,0);
                _gs_send_at(GS_CMD_USOCL,"=i",p0);
                _gs_wait_for_slot();
                //don't check for error on close, can't do much about this (and should always succeed)
                _gs_release_slot(slot);
                slot = NULL;
                err = -1;
            } else err = p0; 
        } else {
            err = -1;
        }
    }
    if(slot) _gs_release_slot(slot);
    return err;
}

C_NATIVE(_g350_socket_create){
    NATIVE_UNWARN();
    // GSocket *sock;
    // GSSlot *slot;
    int err = ERR_OK;
    int32_t family;
    int32_t type;
    int32_t proto;
    if (parse_py_args("III", nargs, args, DRV_AF_INET, &family, DRV_SOCK_STREAM, &type, 6 /*tcp*/, &proto) != 3) return ERR_TYPE_EXC;
    if (type != DRV_SOCK_DGRAM && type != DRV_SOCK_STREAM)
        return ERR_TYPE_EXC;
    if (family != DRV_AF_INET)
        return ERR_UNSUPPORTED_EXC;
    proto = (type == DRV_SOCK_DGRAM) ? 17 : 6;

    RELEASE_GIL();
    err = _g350_usocr(proto);
    if(err<0) {
        err = ERR_IOERROR_EXC;
    } else {
        *res = PSMALLINT_NEW(err);
        err = ERR_OK;
    }
    ACQUIRE_GIL();
    
    return err;
}

C_NATIVE(_g350_socket_connect) {
    C_NATIVE_UNWARN();
    int32_t sock,err=ERR_OK;
    GSocket *ssock;
    GSSlot *slot;
    NetAddress addr;
    uint8_t saddr[16];
    uint32_t saddrlen;

    if (parse_py_args("in", nargs, args, &sock, &addr) != 2)
        return ERR_TYPE_EXC;
    *res = MAKE_NONE();
    saddrlen = _gs_socket_addr(&addr,saddr);
    RELEASE_GIL();
    ssock = _gs_socket_get(sock);
    if(!ssock) {
        err = ERR_IOERROR_EXC;    
    } else {
        slot = _gs_acquire_slot(GS_CMD_USOCO,NULL,0,GS_TIMEOUT*30,0);
        _gs_send_at(GS_CMD_USOCO,"=i,\"s\",i",sock,saddr,saddrlen,OAL_GET_NETPORT(addr.port));
        _gs_wait_for_slot();
        if (slot->err) {
            err = ERR_IOERROR_EXC;
        }
        _gs_release_slot(slot);
    }
    //_gs_socket_error(sock);
    ACQUIRE_GIL();
    return err;
}

C_NATIVE(_g350_socket_close) {
    C_NATIVE_UNWARN();
    int32_t sock;
    GSocket *ssock;
    GSSlot *slot;
    int err = ERR_OK;
    int rr;
    if (parse_py_args("i", nargs, args, &sock) != 1)
        return ERR_TYPE_EXC;
    RELEASE_GIL();
    ssock = _gs_socket_get(sock);
    if(!ssock) {
        err = ERR_IOERROR_EXC;    
    } else {
        if(!ssock->to_be_closed) {
            slot = _gs_acquire_slot(GS_CMD_USOCL,NULL,0,GS_TIMEOUT*15,0);
            _gs_send_at(GS_CMD_USOCL,"=i",sock);
            _gs_wait_for_slot();
            if (slot->err) {
                //ignore usocl error on already closed sockets (timing issues with uusocl)
                //err = ERR_IOERROR_EXC;
            }
            _gs_release_slot(slot);
        }
        _gs_socket_close(sock);
    }

    ACQUIRE_GIL();
    *res = PSMALLINT_NEW(sock);
    return err;
}

C_NATIVE(_g350_socket_send) {
    C_NATIVE_UNWARN();
    uint8_t *buf;
    int32_t len;
    int32_t flags;
    int32_t sock;
    int32_t wrt;
    int32_t tsnd;
    int err = ERR_OK;
    GSocket *ssock;
    GSSlot *slot;
    if (parse_py_args("isi", nargs, args,
                &sock,
                &buf, &len,
                &flags) != 3) return ERR_TYPE_EXC;
    RELEASE_GIL();
    ssock = _gs_socket_get(sock);
    if(!ssock || ssock->to_be_closed) {
        err = ERR_IOERROR_EXC;    
    } else {
        wrt=0;
        while(wrt<len && err==ERR_OK){
            slot = _gs_acquire_slot(GS_CMD_USOWR,NULL,16,GS_TIMEOUT*10,1);
            tsnd = MIN(MAX_SOCK_HEX_BUF/4,(len-wrt));
            _gs_socket_bin_to_hex(buf+wrt,ssock->txbuf,tsnd);
            _gs_send_at(GS_CMD_USOWR,"=i,i,\"s\"",sock,tsnd,ssock->txbuf,tsnd*2);
            _gs_wait_for_slot();
            if (slot->err) {
                err = ERR_IOERROR_EXC;
            } else {
                if (_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&flags,&tsnd)==2){
                    wrt+=tsnd;
                } else {
                    err = ERR_IOERROR_EXC;
                }
            }
            _gs_release_slot(slot);
        }
    }
    ACQUIRE_GIL();
    *res = PSMALLINT_NEW(wrt);
    return err;
}

C_NATIVE(_g350_socket_sendto){
    C_NATIVE_UNWARN();
    uint8_t* buf;
    int32_t len;
    int32_t flags;
    int32_t sock;
    int32_t wrt=0;
    int32_t err=ERR_OK;
    NetAddress addr;
    uint8_t saddr[16];
    uint32_t saddrlen;
    GSocket *ssock;
    GSSlot *slot;

    if (parse_py_args("isni", nargs, args,
            &sock,
            &buf, &len,
            &addr,
            &flags)
        != 4)
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    saddrlen = _gs_socket_addr(&addr,saddr);
    ssock = _gs_socket_get(sock);
    if(!ssock || ssock->to_be_closed || ssock->proto != 17) {
        err = ERR_IOERROR_EXC;    
    } else {
        wrt=0;
        int tsnd;
        while(wrt<len && err==ERR_OK){
            slot = _gs_acquire_slot(GS_CMD_USOST,NULL,16,GS_TIMEOUT*10,1);
            tsnd = MIN(MAX_SOCK_HEX_BUF/4,(len-wrt));
            _gs_socket_bin_to_hex(buf+wrt,ssock->txbuf,tsnd);
            _gs_send_at(GS_CMD_USOST,"=i,\"s\",i,i,\"s\"",sock,saddr,saddrlen,OAL_GET_NETPORT(addr.port),tsnd,ssock->txbuf,tsnd*2);
            _gs_wait_for_slot();
            if (slot->err) {
                err = ERR_IOERROR_EXC;
            } else {
                if (_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&flags,&tsnd)==2){
                    wrt+=tsnd;
                } else {
                    err = ERR_IOERROR_EXC;
                }
            }
            _gs_release_slot(slot);
        }
    }
    
    ACQUIRE_GIL();

    *res = PSMALLINT_NEW(wrt);
    return err;
}

C_NATIVE(_g350_socket_recv_into){
    C_NATIVE_UNWARN();
    uint8_t* buf;
    int32_t len;
    int32_t sz;
    int32_t flags;
    int32_t ofs;
    int32_t sock;
    int rb;
    int trec;
    int err = ERR_OK;
    int32_t timeout;
    uint8_t *hex;
    GSocket *ssock;
    GSSlot *slot;
    if (parse_py_args("isiiI", nargs, args,
            &sock,
            &buf, &len,
            &sz,
            &flags,
            0,
            &ofs)
        != 5)
        return ERR_TYPE_EXC;
    buf += ofs;
    len -= ofs;
    len = (sz < len) ? sz : len;
    RELEASE_GIL();
    ssock = _gs_socket_get(sock);
    if(!ssock || ssock->to_be_closed) {
        err = ERR_IOERROR_EXC;    
    } else {
        timeout = ssock->timeout ? ssock->timeout:-1;
        rb=0;
        while(rb<len && err==ERR_OK){
            slot = _gs_acquire_slot(GS_CMD_USORD,ssock->rxbuf,MAX_SOCK_HEX_RXBUF,GS_TIMEOUT*10,1);
            trec = MIN(MAX_SOCK_HEX_BUF/4,(len-rb));
            _gs_send_at(GS_CMD_USORD,"=i,i",sock,trec);
            _gs_wait_for_slot();
            if (slot->err) {
                if (rb) err = ERR_IOERROR_EXC; //if nothing read, return error
                else len=rb;
            } else {
                if (_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&flags,&trec)==2){
                    rb+=trec;
                    if(trec){
                        hex = slot->resp;
                        hex+=5+((trec>=10)?1:0); //skip 1 int, 2 commas and another int (possibly two digits) and "
                        buf = _gs_socket_hex_to_bin(hex,buf,trec);
                    } else {
                        _gs_release_slot(slot);
                        slot=NULL;
                        if (_gs_socket_wait_rx(ssock,timeout)==VRES_TIMEOUT){
                            err = ERR_TIMEOUT_EXC;
                        }
                        if(ssock->to_be_closed) len=rb; //exit, returning read bytes
                    }
                } else {
                    _gs_release_slot(slot);
                    break;
                    //err = ERR_IOERROR_EXC;
                }
            }
            if (slot) _gs_release_slot(slot);
        }
    }
   // _gs_socket_error(sock);
    ACQUIRE_GIL();
    *res = PSMALLINT_NEW(rb);
    return err;
}

C_NATIVE(_g350_socket_recvfrom_into){
    C_NATIVE_UNWARN();
    uint8_t* buf;
    int32_t len;
    int32_t sz;
    int32_t flags;
    int32_t ofs;
    int32_t sock;
    int rb;
    int trec;
    int err = ERR_OK;
    int32_t timeout;
    uint8_t *hex;
    uint8_t *saddr;
    uint32_t saddrlen;
    PString *oaddr = NULL;
    int32_t port;
    GSocket *ssock;
    GSSlot *slot;

    if (parse_py_args("isiiI", nargs, args,
            &sock,
            &buf, &len,
            &sz,
            &flags,
            0,
            &ofs)
        != 5)
        return ERR_TYPE_EXC;
    buf += ofs;
    len -= ofs;
    len = (sz < len) ? sz : len;
    RELEASE_GIL();
    ssock = _gs_socket_get(sock);
    if(!ssock || ssock->to_be_closed || ssock->proto != 17) {
        err = ERR_IOERROR_EXC;    
    } else {
        timeout = ssock->timeout ? ssock->timeout:-1;
        rb=0;
        while(!rb && err==ERR_OK){
            slot = _gs_acquire_slot(GS_CMD_USORF,ssock->rxbuf,MAX_SOCK_HEX_RXBUF,GS_TIMEOUT*10,1);
            trec = MIN(MAX_SOCK_HEX_BUF/4,(len-rb));
            _gs_send_at(GS_CMD_USORF,"=i,i",sock,trec);
            _gs_wait_for_slot();
            if (slot->err) {
                err = ERR_IOERROR_EXC;
            } else {
                if (_gs_parse_command_arguments(slot->resp,slot->eresp,"iis",&flags,&trec,&saddr,&saddrlen)==3){
                    rb+=trec;
                    if(trec){
                        //skip to the third "
                        uint8_t *pstart;
                        hex = slot->resp;
                        hex = _gs_advance_to(hex,slot->eresp,"\"");
                        hex++;
                        hex = _gs_advance_to(hex,slot->eresp,"\"");
                        hex++;
                        hex++;
                        pstart = hex;
                        hex = _gs_advance_to(hex,slot->eresp,"\"");
                        _gs_parse_number(pstart,hex-1,&port);
                        hex++;
                        buf = _gs_socket_hex_to_bin(hex,buf,trec);
                        oaddr = pstring_new(saddrlen-2,saddr+1);
                    } else {
                        _gs_release_slot(slot);
                        slot=NULL;
                        if (_gs_socket_wait_rx(ssock,timeout)==VRES_TIMEOUT){
                            err = ERR_TIMEOUT_EXC;
                        }
                        if(ssock->to_be_closed) len=rb; //exit, returning read bytes
                    }
                } else {
                    err = ERR_IOERROR_EXC;
                    _gs_release_slot(slot);
                    break;
                }
            }
            if (slot) _gs_release_slot(slot);
        }
    }
    ACQUIRE_GIL();
    if(err==ERR_OK){
        PTuple* tpl = (PTuple*)psequence_new(PTUPLE, 2);
        PTUPLE_SET_ITEM(tpl, 0, PSMALLINT_NEW(rb));
        PTuple* ipo = ptuple_new(2,NULL);
        PTUPLE_SET_ITEM(ipo,0,oaddr);
        PTUPLE_SET_ITEM(ipo,1,PSMALLINT_NEW(port));
        PTUPLE_SET_ITEM(tpl, 1, ipo);
        *res = tpl;
    }
    return err;
}

C_NATIVE(_g350_socket_setsockopt)
{
    C_NATIVE_UNWARN();
    int32_t sock;
    int32_t level;
    int32_t optname;
    int32_t optvalue;
    int32_t err = ERR_OK;
    GSocket *ssock;
    GSSlot *slot;

    if (parse_py_args("iiii", nargs, args, &sock, &level, &optname, &optvalue) != 4)
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    ssock = _gs_socket_get(sock);
    if(!ssock || ssock->to_be_closed){
        err = ERR_IOERROR_EXC;
    } else {
        if (level==0xffff && optname == 1) {
            //RCV_TIMEO
            ssock->timeout = optvalue;
        } else if(level=0xffff && optname == 8) {
            //KEEPALIVE
            slot = _gs_acquire_slot(GS_CMD_USOSO,NULL,0,GS_TIMEOUT*5,0);
            _gs_send_at(GS_CMD_USOSO,"=i,i,i,i",sock,level,optname,(optvalue)?1:0);
            _gs_wait_for_slot();
            if (slot->err) {
                err = ERR_IOERROR_EXC;
            }
            _gs_release_slot(slot);
        }
    }
    ACQUIRE_GIL();

    *res = MAKE_NONE();
    return ERR_OK;
}


// C_NATIVE(_g350_socket_bind){
//     C_NATIVE_UNWARN();
//     int32_t sock;
//     NetAddress addr;
//     GSocket *ssock;
//     GSSlot *slot;
//     int err = ERR_OK;
//     if (parse_py_args("in", nargs, args, &sock, &addr) != 2)
//         return ERR_TYPE_EXC;
//     RELEASE_GIL();
//     ssock = _gs_socket_get(sock);
//     if(!ssock || ssock->to_be_closed){
//         err = ERR_IOERROR_EXC;
//     } else {
//         slot = _gs_acquire_slot(GS_CMD_USOLI,NULL,0,GS_TIMEOUT*10,0);
//         _gs_send_at(GS_CMD_USOLI,"=i,i",sock,OAL_GET_NETPORT(addr.port));
//         _gs_wait_for_slot();
//         if (slot->err) {
//             err = ERR_IOERROR_EXC;
//         }
//         _gs_release_slot(slot);
//     }
//     ACQUIRE_GIL();
//     *res = MAKE_NONE();
//     return err;
// }


C_NATIVE(_g350_socket_select){
    C_NATIVE_UNWARN();
    int32_t timeout;
    int32_t tmp, i, j, sock = -1;
    uint32_t tstart;
    PObject *tobj;

    if (nargs < 4)
        return ERR_TYPE_EXC;

    PObject* rlist = args[0];
    PObject* wlist = args[1];
    PObject* xlist = args[2];
    PObject* tm = args[3];
    int rls = PSEQUENCE_ELEMENTS(rlist);
    uint8_t *rlready = gc_malloc(rls);

    if (tm == MAKE_NONE()) {
        timeout = -1;
    } else if (IS_PSMALLINT(tm)) {
        timeout = PSMALLINT_VALUE(tm);
    } else
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    i=-1;
    tstart = vosMillis();
    while(1){
        i++;
        if(i>=rls) {
            //reset and check timeout
            i=0;
            if(timeout>=0 && (vosMillis()-tstart>timeout)) break;
            vosThSleep(TIME_U(100,MILLIS)); //sleep a bit
            //TODO: consider using RD URC to signal data ready for each socket
            //and suspend here, avoiding polling (quite cumbersome tough)
        }
        tobj = PSEQUENCE_OBJECTS(rlist)[i];
        sock = PSMALLINT_VALUE(tobj);
        printf("S0 %i\n",sock);
        if(sock>=0&&sock<MAX_SOCKS){
            GSocket *ssock = _gs_socket_get(sock); //get socket again, can be closed by URC
            if(!ssock){
                //consider it not ready
                printf("S1\n");
                rlready[i]=0;
            } else {
                GSSlot *slot = _gs_acquire_slot(GS_CMD_USORD,ssock->rxbuf,MAX_SOCK_HEX_RXBUF,GS_TIMEOUT*10,1);
                _gs_send_at(GS_CMD_USORD,"=i,i",sock,0);
                _gs_wait_for_slot();
                if (slot->err) {
                    printf("S2\n");
                    rlready[i]=0; //consider it not ready
                } else {
                    int flags,trec=0;
                    if (_gs_parse_command_arguments(slot->resp,slot->eresp,"ii",&flags,&trec)==2){
                        if (trec) {
                            printf("S3\n");
                            rlready[i]=1; //consider it ready
                            _gs_release_slot(slot);
                            break;
                        } else {
                            printf("S4\n");
                            rlready[i]=0; //consider it not ready
                        }
                    } else {
                        printf("S5\n");
                        rlready[i]=0; //consider it not ready
                    }
                }
                _gs_release_slot(slot);
            }
        } 
    }    
    

    PTuple* tpl = (PTuple*)psequence_new(PTUPLE, 3);
    //count the number of ready sockets
    tmp = 0;
    for (j = 0; j < rls; j++) {
        if (rlready[j]) tmp++;
    }
    //fill ready socket list
    PTuple *rpl = ptuple_new(tmp,NULL);
    tmp = 0;
    for (j = 0; j < rls; j++) {
        if (rlready[j]) {
            PTUPLE_SET_ITEM(rpl,tmp,PSEQUENCE_OBJECTS(rlist)[j]);
            tmp++;
        }
    }
    PTUPLE_SET_ITEM(tpl,0,rpl);
    //ignore wlist and elist: TODO, add this functionality
    rpl = ptuple_new(0,NULL);
    PTUPLE_SET_ITEM(tpl,1,rpl);
    PTUPLE_SET_ITEM(tpl,2,rpl);
    gc_free(rlready); 
    ACQUIRE_GIL();

    *res = tpl;
    return ERR_OK;
}

/////////////////////SSL/TLS
// ALLOW FOR MAX 1 TLS socket

#define _CERT_NONE 1
#define _CERT_OPTIONAL 2
#define _CERT_REQUIRED 4
#define _CLIENT_AUTH 8
#define _SERVER_AUTH 16

int _gs_tls_config(int opcode,int param,uint8_t *sparam,int sparam_len){
    int err = 0;
    GSSlot *slot = _gs_acquire_slot(GS_CMD_USECPRF,NULL,0,GS_TIMEOUT*5,0);
    if(opcode<0) {
        //delete profile
        _gs_send_at(GS_CMD_USECPRF,"=i",GS_TLS_PROFILE);
    } else if(param>=0) {
        //send integer command
        _gs_send_at(GS_CMD_USECPRF,"=i,i,i",GS_TLS_PROFILE,opcode,param); 
    } else if(sparam!=NULL){
        _gs_send_at(GS_CMD_USECPRF,"=i,i,\"s\"",GS_TLS_PROFILE,opcode,sparam,sparam_len);  
    }
    _gs_wait_for_slot();
    if (slot->err) {
        err = 1;
    }
    _gs_release_slot(slot);
    return err;
}
static const uint8_t *_g350_certnames[] = {
    "zcacerts",
    "zclicert",
    "zclipkey"
};

int _gs_tls_load(int type,uint8_t *cert,uint32_t certlen){
    int err = 0;
    GSSlot *slot = _gs_acquire_slot(GS_CMD_USECMNG,NULL,256,GS_TIMEOUT*20,1);
    _gs_send_at(GS_CMD_USECMNG,"=i,i,\"s\",i",0,type,_g350_certnames[type],8,certlen);
    err = _gs_wait_for_slot_mode(cert,certlen);
    _gs_wait_for_slot();
    if (slot->err) {
        err=1;
    } else if(_gs_parse_command_arguments(slot->resp,slot->eresp,"iiss",NULL,NULL,NULL,NULL,NULL,NULL)!=4){
        err = 1;
    }
    _gs_release_slot(slot);
    return err;
}

int _gs_tls_set(int sock){
    int err = 0;
    GSSlot *slot = _gs_acquire_slot(GS_CMD_USOSEC,NULL,0,GS_TIMEOUT*10,0);
    _gs_send_at(GS_CMD_USOSEC,"=i,i,i",sock,1,GS_TLS_PROFILE);
    _gs_wait_for_slot();
    if (slot->err) {
        err=1;
    }
    _gs_release_slot(slot);
    return err;

}

C_NATIVE(_g350_secure_socket)
{
    C_NATIVE_UNWARN();
    int32_t err = ERR_OK;
    int32_t family = DRV_AF_INET;
    int32_t type = DRV_SOCK_STREAM;
    int32_t proto = 6;
    int32_t sock;
    int32_t i;
    int32_t ssocknum = 0;
    int32_t ctxlen;
    uint8_t* certbuf = NULL;
    uint16_t certlen = 0;
    uint8_t* clibuf = NULL;
    uint16_t clilen = 0;
    uint8_t* pkeybuf = NULL;
    uint16_t pkeylen = 0;
    uint32_t options = _CLIENT_AUTH | _CERT_NONE;
    uint8_t* hostbuf = NULL;
    uint16_t hostlen = 0;

    PTuple* ctx;
    ctx = (PTuple*)args[nargs - 1];
    nargs--;
    if (parse_py_args("III", nargs, args, DRV_AF_INET, &family, DRV_SOCK_STREAM, &type, 6, &proto) != 3){
        return ERR_TYPE_EXC;
    }
    if (type != DRV_SOCK_DGRAM && type != DRV_SOCK_STREAM){
        return ERR_TYPE_EXC;
    }
    if (family != DRV_AF_INET)
        return ERR_UNSUPPORTED_EXC;
    if(proto!=6)
        return ERR_UNSUPPORTED_EXC;

    ctxlen = PSEQUENCE_ELEMENTS(ctx);
    if (ctxlen && ctxlen != 5)
        return ERR_TYPE_EXC;

    if (ctxlen) {
        //ssl context passed
        PObject* cacert = PTUPLE_ITEM(ctx, 0);
        PObject* clicert = PTUPLE_ITEM(ctx, 1);
        PObject* ppkey = PTUPLE_ITEM(ctx, 2);
        PObject* host = PTUPLE_ITEM(ctx, 3);
        PObject* iopts = PTUPLE_ITEM(ctx, 4);
        certbuf = PSEQUENCE_BYTES(cacert);
        certlen = PSEQUENCE_ELEMENTS(cacert);
        clibuf = PSEQUENCE_BYTES(clicert);
        clilen = PSEQUENCE_ELEMENTS(clicert);
        hostbuf = PSEQUENCE_BYTES(host);
        hostlen = PSEQUENCE_ELEMENTS(host);
        pkeybuf = PSEQUENCE_BYTES(ppkey);
        pkeylen = PSEQUENCE_ELEMENTS(ppkey);
        options = PSMALLINT_VALUE(iopts);
    }

    GSocket* sslsock = NULL;
    GSSlot *slot;

    //Max one TLS socket in use
    if (gs.secure_sock_id>=0) return ERR_IOERROR_EXC;

    RELEASE_GIL();
    err = ERR_IOERROR_EXC;
    //REFER TO +USECPRF DOCS
    //delete profile
    if(_gs_tls_config(-1,-1,NULL,0)) goto exit;
    //set minimum version TLS1.0
    if(_gs_tls_config(1,1,NULL,0)) goto exit;
    //automatic cypher
    if(_gs_tls_config(2,0,NULL,0)) goto exit;

    //NOTE: ZERYNTH CERTS END with \0
    
    if(options&_CERT_NONE) {
       //NO CACERT VERIFICATION
        if(_gs_tls_config(0,0,NULL,0)) goto exit;
    } else {
        //REQUIRED OR OPTIONAL AUTH
        if(certlen) {
            //CACERT GIVEN
            //load cacert
            if(_gs_tls_load(0,certbuf,certlen-1)) goto exit;
            //internal cacert name
            if(_gs_tls_config(3,-1,_g350_certnames[0],8)) goto exit;
            if(hostlen) {
                //HOSTNAME GIVEN
                if(_gs_tls_config(0,3,NULL,0)) goto exit;
                //set hostname
                if(_gs_tls_config(4,-1,hostbuf,hostlen)) goto exit;
            } else {
                //HOSTNAME NOT GIVEN
                if(_gs_tls_config(0,1,NULL,0)) goto exit;
            }
        }else {
            //NO CACERT
            if(_gs_tls_config(0,0,NULL,0)) goto exit;
        }
    }
    if(clilen) {
        //internal clicert name
        if(_gs_tls_config(5,-1,_g350_certnames[1],8)) goto exit;  
        //load clicert
        if(_gs_tls_load(1,clibuf,clilen-1)) goto exit;
    }
    if(pkeylen){
        //internal clipkey name
        if(_gs_tls_config(6,-1,_g350_certnames[2],8)) goto exit;  
        //load clipkey
        if(_gs_tls_load(2,pkeybuf,pkeylen-1)) goto exit;
    }

    err = _g350_usocr(6); //TCP
    if(err<0){
        err = ERR_IOERROR_EXC;
    } else {
        int sock = err;
        *res = PSMALLINT_NEW(sock);
        if(!_gs_tls_set(sock)){
            //TLS enabled for sock
            gs.secure_sock_id = sock;
        }    
        err = ERR_OK;
    }

exit:
    ;

    ACQUIRE_GIL();

    return err;
}



/////////////////////DNS

C_NATIVE(_g350_resolve){
    C_NATIVE_UNWARN();
    uint8_t* url;
    uint32_t len;
    uint8_t* saddr;
    uint32_t saddrlen;
    int err = ERR_OK;
    if (parse_py_args("s", nargs, args, &url, &len) != 1)
        return ERR_TYPE_EXC;
    GSSlot *slot;
    RELEASE_GIL();
    slot = _gs_acquire_slot(GS_CMD_UDNSRN,NULL,128,GS_TIMEOUT*70,1);
    _gs_send_at(GS_CMD_UDNSRN,"=i,\"s\"",0,url,len);
    _gs_wait_for_slot();
    if (slot->err) {
        err = ERR_IOERROR_EXC;
    } else {
        if(_gs_parse_command_arguments(slot->resp,slot->eresp,"s",&saddr,&saddrlen)==1){
            *res = pstring_new(saddrlen-2,saddr+1);
        } else {
            err = ERR_IOERROR_EXC;
        }
    }
    _gs_release_slot(slot);
    ACQUIRE_GIL();
    return err;
}


/////////////////////SMS HANDLING

