# -*- coding: utf-8 -*-

import feedbackdecode as fd

def kill_XipppyServer():
    fd.XS_close()
    
if __name__ == '__main__':
    kill_XipppyServer()
    