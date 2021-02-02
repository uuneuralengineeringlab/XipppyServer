# -*- coding: utf-8 -*-
"""
Created on Wed Aug 26 12:33:34 2020

@author: Administrator
"""

# perhaps scheduler? threading?

#import sched, time
#s = sched.scheduler(time.time, time.sleep)
#def do_something(sc): 
#    print("Doing stuff...")
#    # do your stuff
#    s.enter(0.5, 1, do_something, (sc,))
#
#s.enter(0.1, 1, do_something, (s,))
#s.run()

# import threading
# import time

# def printit():
#   print(time.time())

# t = threading.Timer(0.033, printit)
# t.start()

# time.sleep(1)
# t.cancel()




# import threading

# def printit():
#   threading.Timer(0.033, printit).start()
#   print(time.time())

# printit()




from threading import Timer

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
        
from time import sleep
import time

def hello():
    print(time.time())

print("starting...")
rt = RepeatedTimer(0.033, hello) # it auto-starts, no need of rt.start()
try:
    sleep(2) # your long-running job goes here...
finally:
    rt.stop() # better in a try/finally block to make sure the program ends!




# import schedule
# import time

# def job():
#     print(f'{time.clock():3.6f}')

# schedule.every(0.033).seconds.do(job)

# while 1:
#     schedule.run_pending()
#     # time.sleep(1)