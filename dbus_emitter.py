#!/usr/bin/env python
import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging

from RILCommonModules.RILSetup import *
from EpuckCentralizedClient.data_manager import *
from EpuckCentralizedClient.utils import *

schedule = sched.scheduler(time.time, time.sleep)
loop = None

#------------------ Signal Despatch ---------------------------------
class TaskStatusSignal(dbus.service.Object):
    def __init__(self, object_path):
        dbus.service.Object.__init__(self, dbus.SessionBus(), object_path)
    @dbus.service.signal(dbus_interface= DBUS_IFACE_EPUCK, signature='sii')
    def  TaskStatus(self,  sig,  robotid,  taskid):
        #logger.info("Emitted %s : Robot %d now %s ",  sig,  taskid,  status)
        #print "Emitted %s : Robot selected task %d now %s \
        #"  %(sig,  taskid,  status)
        pass
    def Exit(self):
        global loop
        loop.quit()

    
def emit_task_signal(delay,  sig1):
    global task_signal,  datamgr_proxy
    schedule.enter(delay, 0, emit_task_signal, (delay, sig1  ) )
    try:
        datamgr_proxy.mSelectedTaskStarted.wait()
        robotid = datamgr_proxy.mRobotID
        taskdict = datamgr_proxy.mSelectedTask
        datamgr_proxy.mSelectedTaskStarted.clear()
        taskid =  eval(str(taskdict[SELECTED_TASK_ID])) 
        status = str(taskdict[SELECTED_TASK_STATUS]) 
#        for k, v in taskdict.items():
#            taskid = eval(str(k))
#            status = str(v)
        print "From TaskDict got %i %s"  %(taskid,  status)
        task_signal.TaskStatus(sig1,  robotid,  taskid)
    except:
        print "Emitting TaskStatus signal failed"
   


def emitter_main(dm,  dbus_iface= DBUS_IFACE_EPUCK,  dbus_path =\
    DBUS_PATH_BASE, sig1 = "TaskStatus",   delay = 3):
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    session_bus = dbus.SessionBus()
    global task_signal,  datamgr_proxy
    datamgr_proxy = dm
    try:
        name = dbus.service.BusName(dbus_iface, session_bus)
        task_signal = TaskStatusSignal(dbus_path)
        loop = gobject.MainLoop()
        print "Running Outbound TaskStatus service."
    except dbus.DBusException:
        traceback.print_exc()
        sys.exit(1)
    try:
            e = schedule.enter(0, 0, emit_task_signal, (delay,  sig1,  ))
            schedule.run()
            loop.run()
    except (KeyboardInterrupt, dbus.DBusException, SystemExit):
            print "User requested exit... shutting down now"
            task_signal.Exit()
            pass
            sys.exit(0)
