"""
.. module:: g350

***********
G350 Module
***********

This module implements the Zerynth driver for the Ublox G350 (or U260) gsm/gprs chip (`System Integration Manual <https://www.u-blox.com/sites/default/files/SARA-G3-U2_SysIntegrManual_(UBX-13000995).pdf>`_).

The driver must be used together with the standard library :ref:`GSM Module <stdlib_gsm>`.

The following functionalities are implemented:

    * attach/detach from gprs network
    * retrieve and set available operators
    * retrieve signal strength
    * retrieve network and device info
    * socket abstraction (and secure socket if available on the model). Listening sockets for TCP and UDP protocols are not implemented due to the nature of GSM networks.

The communication with G350 is performed via UART without hardware flow control.

This module provides the :samp:`g350Exception` to signal errors related to the hardware initialization and management.

   """



new_exception(g350Exception, Exception)

def auto_init():
    """
.. function::auto_init()

    Tries to initialize the G350 device with auto parameters if possible.
    Raise :samp:`UnsupportedError` otherwise.
    """
    if __defined(BOARD, "particle_electron"):
        init(SERIAL5,D35,D32,D33,D34)
    else:
        raise UnsupportedError

def init(serial,dtr,rts,poweron,reset):
    """
.. function::init(serial,dtr,rts,poweron,reset)

    Initialize the G350 device given the following parameters:

    * *serial*, the serial port connected to the G350 (:samp;`SERIAL1`,:samp:`SERIAL2`, etc..)
    * *dtr*, the DTR pin of G350
    * *rts*, the RTS pin of G350
    * *poweron*, the power up pin of G350
    * *reset*, the reset pin of G350

    """
    _init(serial,dtr,rts,poweron,reset,__nameof(g350Exception))
    __builtins__.__default_net["gsm"] = __module__
    __builtins__.__default_net["ssl"] = __module__
    __builtins__.__default_net["sock"][0] = __module__ #AF_INET

@c_native("_g350_init",["csrc/g350.c"])
def _init(serial,dtr,rst,poweron,reset,exc):
    pass


@c_native("_g350_attach",["csrc/g350.c"])
def attach(apn,username,password,authmode,timeout):
    pass

@c_native("_g350_detach",["csrc/g350.c"])
def detach():
    pass


@c_native("_g350_network_info",["csrc/g350.c"])
def network_info():
    pass

@c_native("_g350_mobile_info",["csrc/g350.c"])
def mobile_info():
    pass

@c_native("_new_check_network",["csrc/g350.c"])
def check_network():
    pass

@c_native("_g350_link_info",["csrc/g350.c"])
def link_info():
    pass

@c_native("_g350_operators",["csrc/g350.c"])
def operators():
    pass

@c_native("_g350_set_operator",["csrc/g350.c"])
def set_operator(opname):
    pass

@c_native("_g350_last_error",["csrc/g350.c"])
def last_error():
    """
.. function::last_error()

    Return the textual description of the last AT command error (refer to +CME ERRORS in the `AT command manual <www.u-blox.com/sites/default/files/u-blox-ATCommands_Manual_(UBX-13002752).pdf>`_

    """
    pass

@native_c("_g350_rtc",["csrc/g350.c"])
def rtc():
    """
    
.. function:: rtc()

    Return a tuple of seven elements:

        * current year
        * current month (1-12)
        * current day (1-31)
        * current hour (0-23)
        * current minute (0-59)
        * current second (0-59)
        * current timezone in minutes away from GMT 0

    The returned time is always UTC time with a timezone indication.

    """
    pass

@c_native("_g350_rssi",["csrc/g350.c"])
def rssi():
    pass

@native_c("_g350_resolve",["csrc/*"])
def gethostbyname(hostname):
    pass


@native_c("_g350_socket_create",["csrc/*"])
def socket(family,type,proto):
    pass

@native_c("_g350_socket_setsockopt",["csrc/*"])
def setsockopt(sock,level,optname,value):
    pass


@native_c("_g350_socket_close",["csrc/*"])
def close(sock):
    pass


@native_c("_g350_socket_sendto",["csrc/*"])
def sendto(sock,buf,addr,flags=0):
    pass

@native_c("_g350_socket_send",["csrc/*"])
def send(sock,buf,flags=0):
    pass

def sendall(sock,buf,flags=0):
    send(sock,buf,flags)


@native_c("_g350_socket_recv_into",["csrc/*"])
def recv_into(sock,buf,bufsize,flags=0,ofs=0):
    pass


@native_c("_g350_socket_recvfrom_into",["csrc/*"])
def recvfrom_into(sock,buf,bufsize,flags=0):
    pass


def bind(sock,addr):
    raise UnsupportedError

def listen(sock,maxlog=2):
    raise UnsupportedError

def accept(sock):
    raise UnsupportedError

@native_c("_g350_socket_connect",["csrc/*"])
def connect(sock,addr):
    pass

@native_c("_g350_socket_select",[])
def select(rlist,wist,xlist,timeout):
    pass

@native_c("_g350_secure_socket",[],[])
def secure_socket(family, type, proto, ctx):
    pass

