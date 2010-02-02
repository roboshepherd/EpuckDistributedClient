#!/usr/bin/env python

#from device_controller import *

from epuck_navigator import *

navigator = EpuckNavigator()
epuck = Epuck(12)

#dc = DeviceController(1,0)
#print "TCA:", dc.navigator.mTaskConeAngle

navigator.GoForward(epuck, 10)
#navigator.GoTowardsTarget(epuck,  2429, 680, 5.39, 500, 2530, 10)
