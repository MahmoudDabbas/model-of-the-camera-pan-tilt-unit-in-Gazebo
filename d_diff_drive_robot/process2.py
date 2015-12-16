import ach
import sys
import time
import ipc

DataP1 = ipc.E_PY()
CHAN_1 = ach.Channel(ipc.EXAMPLE_CHAN_1)
CHAN_2 = ach.Channel(ipc.EXAMPLE_CHAN_2)
CHAN_2.flush()

Time = time.time()
while True:
	CHAN_1.get(DataP1,wait=True,last=True)
#	DataP2.Value[0] = 2.5*DataP1.Value[0]
#	CHAN_2.put(DataP2)
#        print "2.5*DataP1.Value[0] : ",2.5*DataP1.Value[0]
	print "Pitch = : ",DataP1.P[0]
	print "Yaw = : ",DataP1.Y[0]

