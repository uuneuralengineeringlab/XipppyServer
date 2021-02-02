# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 10:18:54 2020

@author: Administrator
"""

import threading, socket, numpy as np, time

p = threading.Event()
udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp.bind(("127.0.0.1", 20001))
udp.setblocking(0)

def main():
    while not p.wait(1):
        print('hello')
        time.sleep(0.3)
        s = "SS="+np.array2string(np.random.rand(5))+";"
        udp.sendto(str.encode(s),("127.0.0.1",20002))
        try:
            data = udp.recv(1)
        except:
            data = b'\x00'
        if (data==b'\x01'):
            p.set()

t = threading.Thread(target=main)
t.start()

t.join() #waits here until thread finishes
print('finished')

udp.close()
