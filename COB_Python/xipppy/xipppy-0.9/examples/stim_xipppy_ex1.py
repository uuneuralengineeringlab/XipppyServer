import xipppy as xp
import time

elec=0
res=3

xp._open(use_tcp=True)

stimList = xp.list_elec('stim')

if xp.stim_get_res(0) <= 1: #change stim res
	xp.stim_enable_set(False)
	xp.stim_set_res(elec,res)

xp.stim_enable_set(True)

pseg=xp.StimSegment(6,100,-1) #create first phase (cathodic) segment for stimulation.
#duration of 200 us (6 clock cycles at 30 kHz), and ampltidue of 100, and negative polarity

ipi=xp.StimSegment(3,0,1) #creates inter-phase-interval. 
#duration of 100 us (3 clock cycles at 30 kHz), amplitude of 0, enable set to false.

nseg=xp.StimSegment(6,100,1,delay=31) #creates second, anodic phase.
#duration of 200 us (6 cycles at 30 kHz), and ampltidue of 100, and positive polarity


seq0=xp.StimSeq(elec, 1000,1,pseg,ipi,nseg)#create overall sequence header defining electrode, period, repeats
#This will stimulate on electrode 1 at 30 hz for one second (1000 ms)

xp.StimSeq.send(seq0) #send sequence to NIP

time.sleep(0.1)

xp._close()
