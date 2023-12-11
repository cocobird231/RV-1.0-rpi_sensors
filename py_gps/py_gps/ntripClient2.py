import sys
import time
import datetime
import socket
import base64
import threading

import serial
from pynmeagps import NMEAReader

version=0.1
useragent="NTRIP MIRDCRVTestClient/%.1f" % version

gpsDict = { 'status' : 0, 'lat' : 0, 'lon' : 0, 'alt' : 0 }
ros2DictLock = threading.Lock()

'''
NMEA 0183 Data Structure
https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html
'''

class NtripClient(object):
    def __init__(self, 
                    device : str = "/dev/ttyS0", 
                    baud : int = 38400, 
                    caster : str = "", 
                    port : int = 2101, 
                    mountpoint : str = "/", 
                    user : str = "name:passwd", 
                    useV2 : bool = False):
        # Ntrip
        self.__device = device
        self.__baud = baud
        self.__caster = caster
        self.__port = port
        self.__mountpoint = mountpoint if (len(mountpoint) > 0 and mountpoint[0] == '/') else '/' + mountpoint
        self.__user = base64.b64encode(bytes(user,'utf-8')).decode("utf-8")
        self.__v2 = useV2

        # GNSS
        self.__gnssStream = serial.Serial(device, baud, timeout=3)
        self.__nmeaReader = NMEAReader(self.__gnssStream)
        self.__stopF = False

        self.__buffSize = 50
        self.__currentGGA = None# Byte arr of GNGGA data

        # Run threads
        self.__gnssTh = threading.Thread(target=self.__getNMEA, daemon=True)
        self.__gnssTh.start()
        self.__ntripTh = threading.Thread(target=self.__getRTCM, daemon=True)
        self.__ntripTh.start()
    
    def __del__(self):
        self.__stopF = True
        self.__ntripTh.join()
        self.__gnssTh.join()

    def __getRTCM(self):
        # HTTP request form
        httpGetStr = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (self.__mountpoint, useragent, self.__user)
        httpGetStr += "Host: %s:%i\r\n" %(self.__caster, self.__port)
        if (self.__v2) : httpGetStr += "Ntrip-Version: Ntrip/2.0\r\n"
        print(httpGetStr)
        httpGetBStr = bytes(httpGetStr, 'ascii')

        ntripSock = None
        prePos = None# Char array of GNGGA data

        while (not self.__stopF):
            # Establish ntrip conn
            if (ntripSock == None):
                # Create socket
                try:
                    ntripSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    ret = ntripSock.connect_ex((self.__caster, self.__port))

                    # Socket return succeed
                    if (ret == 0):
                        print(ret)
                        ntripSock.settimeout(5)
                        ntripSock.sendall(httpGetBStr)

                        casterResBStr = ntripSock.recv(4096) #All the data
                        casterResStrList = casterResBStr.decode('ascii').split("\r\n")
                        print(casterResStrList)

                        connF = False
                        for i in casterResStrList:
                            if (len(i) <= 0) : continue
                            elif (i == 'ICY 200 OK'):
                                print('Mountpoint checked.')
                                connF = True
                            elif (i == 'SOURCETABLE 200 OK'):
                                print('Mountpoint not found.')

                        if (not connF and ntripSock):
                            ntripSock.shutdown(socket.SHUT_RDWR)
                            ntripSock.close()
                            ntripSock = None

                    # Socket return failed
                    else:
                        print('Socket error:' + ret)
                        if (ntripSock):
                            ntripSock.shutdown(socket.SHUT_RDWR)
                            ntripSock.close()
                            ntripSock = None

                except Exception as e:
                    print('\nException' + e)
                    if (ntripSock):
                        ntripSock.shutdown(socket.SHUT_RDWR)
                        ntripSock.close()
                        ntripSock = None
            
            # Ntrip working
            else:
                if (prePos != self.__currentGGA):
                    # Send NMEA to ntrip server and recv RTCM.
                    # Send RTCM to gnss module.
                    # ntripSock set to None if send or recv error.
                    try:
                        ntripSock.sendall(self.__currentGGA)
                        recvRTCM = ntripSock.recv(self.__buffSize)
                        self.__gnssStream.write(recvRTCM)
                        prePos = self.__currentGGA
                    except Exception:
                        print('Ntrip socket send or recv error.')
                        if (ntripSock):
                            ntripSock.shutdown(socket.SHUT_RDWR)
                            ntripSock.close()
                            ntripSock = None

            time.sleep(1)

        # Outside loop. Clear socket and return.
        if (ntripSock):
            ntripSock.shutdown(socket.SHUT_RDWR)
            ntripSock.close()
            ntripSock = None

    def __getNMEA(self):
        while (not self.__stopF):
            try:
                nmeaBStr, _ = self.__nmeaReader.read()
                if (bytes("GNGGA",'ascii') in  nmeaBStr):
                    self.__currentGGA = nmeaBStr
                    contentList = nmeaBStr.decode('ascii').split(',')# List of GNGGA content
                    if (len(contentList) >= 15 and contentList[0] == '$GNGGA'):
                        '''
                        GNGGA data: <$GNGGA>, 
                                    <UTC time>, 
                                    <Latitude>, 
                                    <Latitude hemisphere>, 
                                    <Longitude>, 
                                    <Longitude hemisphere>, 
                                    <Status>, 
                                    <Satellites>, 
                                    <HDOP level precision factor>, 
                                    <Altitude>, 
                                    <M>, 
                                    <Altitude geoid>, 
                                    <M>, 
                                    <Differential time>, 
                                    <Differential reference base station label and check value>
                        '''
                        lat = float(contentList[2][:2]) + float(contentList[2][2:]) / 60# DMM to DD
                        lon = float(contentList[4][:3]) + float(contentList[4][3:]) / 60# DMM to DD
                        if (contentList[3] == 'S') : lat = -lat
                        if (contentList[5] == 'W') : lon = -lon
                        
                        status = int(contentList[6])
                        alt = float(contentList[9])

                        ros2DictLock.acquire()
                        gpsDict['status'] = status
                        gpsDict['lat'] = lat
                        gpsDict['lon'] = lon
                        gpsDict['alt'] = alt
                        ros2DictLock.release()

            except Exception:
                print('Read GNGGA error')
            
            time.sleep(0.05)

        if (self.__gnssStream):
            self.__gnssStream.close()

    def close(self):
        self.__stopF = False
