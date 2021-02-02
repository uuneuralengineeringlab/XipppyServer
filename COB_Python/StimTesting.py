# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 08:29:56 2020

@author: Administrator
"""

import xipppy as xp
import time
import numpy as np

xp._open()

# use dir(xp) and help(xp.method) to find methods and get the docs for them

###################### Enabling and checking stim ############################
xp.stim_enable_set(True) # should enable stim... seems to work based on next check
xp.stim_enable() # this checks if stim is enabled.

############### Building Stim Segments #######################################
stim_segs = []
stim_segs.append(xp.StimSegment(length=6, amplitude=50, polarity=-1))
stim_segs.append(xp.StimSegment(length=3, amplitude=0,   polarity=-1))
stim_segs.append(xp.StimSegment(length=6, amplitude=50, polarity= 1))

################ Stim Segments -> Stim Sequence ##############################
freq = 100 #Hz
period = int(np.floor(30000/freq))
repeats = int(np.ceil(0.033*freq))
act = 4

stim_seqs = []
stim_seqs.append(xp.StimSeq(0, period, repeats, *stim_segs, action=act)) #electrode, period, repeats
stim_seqs.append(xp.StimSeq(1, period, repeats, *stim_segs, action=act))
stim_seqs.append(xp.StimSeq(2, period, repeats, *stim_segs, action=act))
stim_seqs.append(xp.StimSeq(3, period, repeats, *stim_segs, action=act))


################## Send to NIP? ##############################################
# seq.send() # for single electrode

while True:
    xp.StimSeq.send_stim_seqs(stim_seqs)
    time.sleep(0.033)
