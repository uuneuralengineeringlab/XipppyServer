# -*- coding: utf-8 -*-
"""
Created on Wed Aug 26 11:04:29 2020

@author: Administrator
"""

import threading
import time
from multiprocessing.pool import ThreadPool

def thread_fun():
    for i in range(5):
        print('in this thread\n')
        time.sleep(1)
    return 'Done'

#th = threading.Thread(target=thread_fun)

#th.start()

#for i in range(5):
#    print('in main thread\n')
#    time.sleep(1)
    
pool = ThreadPool(processes=1)

output = ''
while not output:
    print('in main loop')
    output = pool.apply_async(thread_fun)
    print(output)
#    output = output.get()
    time.sleep(1)
    
    
    
###############################################################################
from queue import Queue 
from threading import Thread 
  
# A thread that produces data 
def producer(out_q): 
    while True: 
        # Produce some data 
        ... 
        out_q.put(data) 
          
# A thread that consumes data 
def consumer(in_q): 
    while True: 
        # Get some data 
        data = in_q.get()
        print(data)
        # Process the data 
        if in_q.qsize() > 0:
#            data = in_q.get() 
            print(f'q_size {in_q.qsize()}')
            print(data)
        ... 
          
# Create the shared queue and launch both threads 
q = Queue() 
t1 = Thread(target = consumer, args =(q, )) 
#t2 = Thread(target = producer, args =(q, )) 
t1.start() 
#t2.start() 
data = 'hi'

q.put(data)

q.qsize()
q.get()

t1.join()

