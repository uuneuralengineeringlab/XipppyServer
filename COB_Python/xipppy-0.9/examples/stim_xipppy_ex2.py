import xipppy as xp
import time
import math

clock_cycle=1/30*1000 #single clock cycle in microseconds
delay_length=1/30*1000/32 #calculate the length of a single unit of delay in microseconds
elec=0 #electrode num
res=3 #resolution (10 uA/step)

xp._open(use_tcp=True)

stimList = xp.list_elec('stim')

if xp.stim_get_res(0) <= 1: #change stim res
	xp.stim_enable_set(False)
	xp.stim_set_res(elec,res)

xp.stim_enable_set(True)

seg1=xp.StimSegment(1,100,-1) #seg1: 1 clock cycle worth of stim for the first 50 us phase

cath_remaining=50-clock_cycle #calculate remianing duration of the cathodic phase
cath_delay=math.floor(cath_remaining/delay_length) #actual delay param must be integer between 0-31

seg2=xp.StimSegment(1,100,-1,enable=False,delay=cath_delay) #seg 2: full cathodic phase and a bit of interphase interval. 'enable' field set to zero, sets pulse to start high and transition to off

seg3=xp.StimSegment(2,0,1,enable=False) #seg3: interphase interval is 100 us, so add two more full clock cycles worth of stim for 66.6 us more

ipi_remaining=100-(clock_cycle-cath_delay*delay_length)-2*clock_cycle #calculate how much more of inter-phase is remaining, being careful to account for quantization of delay at the end of cathodic
ipi_delay=math.floor(ipi_remaining/delay_length)

seg4=xp.StimSegment(1,100,1,delay=ipi_delay) #seq4: enable field sets the pulse to start off and transition to on

anod_remaining=50-(clock_cycle-ipi_delay*delay_length) #finish pulse with one use of delay field
anod_delay=math.floor(anod_remaining/delay_length)

seg5=xp.StimSegment(1,100,1,enable=False, delay=anod_delay) #seg5: last command word

seq0=xp.StimSeq(elec,1000,30,seg1,seg2,seg3,seg4,seg5)#create overall sequence header defining electrode, period, repeats, and segments
#This will stimulate on electrode 1 at 30 hz for one second (1000 ms)

xp.StimSeq.send(seq0) #send sequence to NIP

time.sleep(0.1) #why do i need this
xp._close()
