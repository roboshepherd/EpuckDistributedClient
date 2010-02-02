#!/usr/bin/env python
import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging

from RILCommonModules.RILSetup import *
from RILCommonModules.pose import *

schedule = sched.scheduler(time.time, time.sleep)
pose = Pose(x=700,  y=500,  theta=2.5).info

class TrackerSignal(dbus.service.Object):
    def __init__(self, object_path):
        dbus.service.Object.__init__(self, dbus.SessionBus(), object_path)
        #global taskinfo
    @dbus.service.signal(dbus_interface= DBUS_IFACE_TRACKER,\
            signature='sa{sd}')
    def RobotPose(self, sig,  pose):
        # The signal is emitted when this method exits
        #pass
        print "Tracker signal: %s" %sig
        print pose
    def Exit(self):
        loop.quit()

#Emit DBus-Signal
def emit_tracker_signal(sig1,  inc):
    print "At emit_tracker_signal():"
    #taskinfo.Print()
    global tracker_signal,  pose
    schedule.enter(inc, 0, emit_tracker_signal, (sig1,   inc)) # re-schedule to repeat this function
    print "\tEmitting signal>>> " 
    tracker_signal.RobotPose(sig1,  pose)

def server_main(dbus_iface= DBUS_IFACE_TRACKER,\
        dbus_path = DBUS_PATH_BASE, sig1 = "RobotPose",   delay = 1):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        session_bus = dbus.SessionBus()
        global tracker_signal
        try:
            name = dbus.service.BusName(dbus_iface, session_bus)
            tracker_signal = TrackerSignal(dbus_path)
            loop = gobject.MainLoop()
            print "Running example signal emitter service."
        except dbus.DBusException:
            traceback.print_exc()
            sys.exit(1)
        try:
                e = schedule.enter(0, 0, emit_tracker_signal, (sig1,   delay,  ))
                schedule.run()
                loop.run()
        except (KeyboardInterrupt, dbus.DBusException, SystemExit):
                print "User requested exit... shutting down now"
                tracker_signal.Exit()
                pass
                sys.exit(0)
