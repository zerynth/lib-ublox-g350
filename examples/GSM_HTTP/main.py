################################################################################
# HTTP Time GSM Example
#
# Created: 2016-07-27 11:00:55.020628
#
################################################################################

# the classic wifi requests example, with very little changes can access
# the net through a gsm connection!

# import our gsm chip specific driver
from ublox.g350 import g350
# and the generic gsm module
from wireless import gsm
import streams
import requests
import json


streams.serial()

try:
    # init the gsm driver!
    # The driver automatically registers itself to the gsm interface
    # with the correct configuration for the selected device
    # NOTE: change this line to g350.init(...) with correct parameters
    #       if not running the example on a Particle Electron board
    g350.auto_init()
    print("Registering to the network...")
    
    # connect to our APN
    for i in range(20):
        try:
            # set here the APN name
            gsm.attach('spark.telefonica.com')
            break
        except g350Exception:
            print("Something wrong on the G350")
        except TimeoutError:
            print("Can't register to network, took too long")
        sleep(2000)    
    else:
        print("ooops,  can't register at all!")
        while True:
            sleep(1000)

    print("Signal Strength:",gsm.rssi())
    print("Link info",gsm.link_info())
    print("Network info",gsm.network_info())
    print("Device info",gsm.mobile_info())

    # from now on everything is exactly identical to wifi HTTP Time Example ;)

    # let's try to connect to timeapi.org to get the current UTC time
    for i in range(3):
        try:
            print("Trying to connect...")
            # we need to impersonate a web browser: as easy as setting the http user-agent header
            user_agent = {"user-agent":"Mozilla/5.0 (iPad; U; CPU OS 3_2_1 like Mac OS X; en-us) AppleWebKit/531.21.10 (KHTML, like Gecko) Mobile/7B405"}
            # go get that time!
            # url resolution and http protocol handling are hidden inside the requests module
            response = requests.get("http://www.worldtimeserver.com/handlers/GetData.ashx",{"action":"GCTData"},headers=user_agent)
            # let's check the http response status: if different than 200, something went wrong
            print("Http Status:",response.status)
            # if we get here, there has been no exception, exit the loop
            break
        except Exception as e:
            print(e)


    try:
        # check status and print the result
        if response.status==200:
            print("Success!!")
            print("-------------")
            print("And the result is:",response.content)
            print("-------------")
            js = json.loads(response.content)
            print("City:",js["City"])
            print("Date:",js["FormattedDate"])
            print("Time:",js["ThisTime"])
    except Exception as e:
        print("ooops, something very wrong! :(",e)
except Exception as e:
    print("Something bad happened",e)
