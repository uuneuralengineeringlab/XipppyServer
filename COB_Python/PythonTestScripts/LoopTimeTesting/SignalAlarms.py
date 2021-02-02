# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 08:43:15 2020

@author: Administrator
"""

import signal
import os
import time

def handler(signum, frame):
    print('Signal handler called with signal', signum)
    time.sleep(0.1)
    # signal.raise_signal(signal.SIGUSR1)
    
def completer(signum, frame):
    print('Completion handler called with signal', signum)
    
signal.signal(signal.SIGALRM, handler)
signal.signal(signal.SIGUSR1, completer)

signal.setitimer(signal.ITIMER_REAL, 2, 0.5)

while True:
    print('Waiting...')
    time.sleep(3)
    signal.sigwait(signal.SIGALRM)
    
    
signal.alarm(0)