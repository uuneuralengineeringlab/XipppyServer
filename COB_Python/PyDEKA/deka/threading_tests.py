import time
import threading

class TestLoop():
    def __init__(self):
        self.value = 0
        self.thread = threading.Thread(target=self.run)
        
    def start(self):
        self.thread.start()
        
    def run(self):
        self.basetime = time.time() 
        while True:
            self.value = int(time.time())

t = TestLoop()
t.start()

if __name__ == '__main__':
    t = TestLoop()
    t.start()
    
    
