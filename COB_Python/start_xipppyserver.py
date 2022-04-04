# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 10:56:21 2021

@author: Administrator
"""

import subprocess

while True:
    try:
        subprocess.check_call(['/usr/rppl/www/bin/python3', 'XipppyServer.py'])
        break
    except:
        print('Failed to start XipppyServer')