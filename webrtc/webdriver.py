#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import http.server
from http.server import SimpleHTTPRequestHandler

from selenium import webdriver
from bs4 import BeautifulSoup
import threading
import time

localhost = '127.0.0.1'
DIRECTORY = '/home/pi/webcam'

class HTTP_Header(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)



def start_server(port):
    #HandlerClass = SimpleHTTPRequestHandler
    HandlerClass = HTTP_Header
    ServerClass  = http.server.HTTPServer
    Protocol     = "HTTP/1.1"

    server_address = (localhost, port)

    HandlerClass.protocol_version = Protocol
    httpd = ServerClass(server_address, HandlerClass)
    sa = httpd.socket.getsockname()
    print("Serving HTTP on", sa[0], "port", sa[1], "...")
    httpd.serve_forever()

def openWebDriver(url):
    try:
        print('[openWebDriver]: url: %s' %url)
        options = webdriver.ChromeOptions()
        #options.add_argument('--headless')
        options.add_argument('--use-fake-ui-for-media-stream')
        browser = webdriver.Chrome(options = options)
        browser.set_page_load_timeout(60)
        browser.get(url)
        while (browser):
            html = browser.page_source
            soup = BeautifulSoup(html, 'lxml')
            status = soup.find(id = 'status').text
            print(status)
            if (status == 'Too many connection attempts, aborting. Refresh page to try again'):
                browser.refresh()
            time.sleep(5)

       
    except Exception as e:
        print('[ERROR]: %s' %str(e))
        return

if __name__ == '__main__':
    if sys.argv[1:]:
        port = int(sys.argv[1])
    else:
        port = 8000
    # Open server thread
    serverTH = threading.Thread(target = start_server, args = (port, ))
    serverTH.start()

    # Open webdriver
    openWebDriver('http://%s:%d' %(localhost, port))

    serverTH.join()
