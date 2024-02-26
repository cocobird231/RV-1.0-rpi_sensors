import threading
import socket
import sys
import datetime
import base64
import time
#import ssl
from optparse import OptionParser

import serial
from pynmeagps import NMEAReader

version=0.2
useragent="NTRIP MIRDCRVTestClient/%.1f" % version

caster = "61.220.23.239" # "rtk2go.com"
port = 2101
mountpoint = "/MIRDC01" # "/NCKU-meclab"z
username = "mirdc" # "123@gmail.com"
passwd = "none" # "none"
user = base64.b64encode(bytes("%s:%s" %(username, passwd),'utf-8')).decode("utf-8")

httpGetStr = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (mountpoint, useragent, user)
httpGetStr += "Host: %s:%i\r\n" %(caster, port)
# httpGetStr += "Ntrip-Version: Ntrip/2.0\r\n"
print(httpGetStr)

httpGetBStr = bytes(httpGetStr, 'ascii')

try:
    ntripSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ret = ntripSock.connect_ex((caster, port))

    if (ret == 0):
        print(ret)
        ntripSock.settimeout(5)
        ntripSock.sendall(httpGetBStr)

        casterResBStr = ntripSock.recv(4096) #All the data
        casterResStr = casterResBStr.decode('ascii').split("\r\n")
        print(casterResStr)

        ntripSock.shutdown(socket.SHUT_RDWR)
        ntripSock.close()
    else:
        print(ret)

except Exception as e:
    print('\nException')
    print(e)