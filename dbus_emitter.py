#!/usr/bin/env python
import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging

from RILCommonModules.RILSetup import *
from EpuckDistributedClient.data_manager import *
from EpuckDistributedClient.utils import *

schedule = sched.scheduler(time.time, time.sleep)
loop = None

#------------------ Signal Despatch ---------------------------------
class RobotTaskEngagementSignal(dbus.service.Object):
    def __init__(self, object_path):
        dbus.service.Object.__init__(self, dbus.SessionBus(), object_path)
    @dbus.service.signal(dbus_interface= DBUS_IFACE_EPUCK, signature='sii')
    def  EmitRobotTask(self,  sig,  robotid,  taskid):
        #logger.info("Emitted %s : Robot %d now %s ",  sig,  taskid,  status)
        #print "Emitted %s : Robot selected task %d now %s \
        #"  %(sig,  taskid,  status)
        pass
    def Exit(self):
        global loop
        loop.quit()

class LocalTaskInfoSignal(dbus.service.Object):
    def __init__(self, object_path):
        dbus.service.Object.__init__(self, dbus.SessionBus(), object_path)
    @dbus.service.signal(dbus_interface= DBUS_IFACE_EPUCK,\
     signature='ia{iad}')
    def  EmitLocalTaskInfo(self,  robotid,  taskinfo):
        #logger.info("Emitted %s : Robot %d now %s ",  sig,  taskid,  status)
        print "LocalTaskInfo emitted from robot%i : local taskinfo: %s"\
         %(robotid,  taskinfo)
        pass
    def Exit(self):
        global loop
        loop.quit()

    
def emit_robot_signals(delay,  sig1):
    global task_signal,  datamgr_proxy, local_signal
    # reschedule 
    schedule.enter(delay, 0, emit_robot_signals, (delay, sig1  ) )
    # emit robot's local taskinfo
    try:
        robotid = datamgr_proxy.mRobotID
        taskinfo = datamgr_proxy.mLocalTaskInfo.copy()
        local_signal.EmitLocalTaskInfo(robotid,  taskinfo)
    except Exception, e:
        print "Err at emit_robot_signals():", e
    # emit robot's task activity signal
    try:
        datamgr_proxy.mSelectedTaskStarted.wait() ### Blocking !!! ###
        robotid = datamgr_proxy.mRobotID
        taskdict = datamgr_proxy.mSelectedTask
        datamgr_proxy.mSelectedTaskStarted.clear()
        taskid =  eval(str(taskdict[SELECTED_TASK_ID])) 
        status = str(taskdict[SELECTED_TASK_STATUS]) 
        #print "From TaskDict got %i %s"  %(taskid,  status)
        task_signal.EmitRobotTask(sig1,  robotid,  taskid)
    except:
        print "Emitting Robot Task Status signal failed"
   

def emitter_main(dm,  dbus_iface= DBUS_IFACE_EPUCK,  dbus_path1 =\
    DBUS_PATH_BASE, dbus_path2 = DBUS_PATH_EPUCK_LOCALITY,\
    sig1 = SIG_TASK_STATUS, sig2 = SIG_LOCAL_TASK_INFO, delay = 3):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        session_bus = dbus.SessionBus()
        global task_signal,  datamgr_proxy, local_signal
        datamgr_proxy = dm
        try:
            name = dbus.service.BusName(dbus_iface, session_bus)
            task_signal = RobotTaskEngagementSignal(dbus_path1)
            local_signal = LocalTaskInfoSignal(dbus_path2)
            loop = gobject.MainLoop()
            print "Running Outbound Robot Signalling services."
        except dbus.DBusException:
            traceback.print_exc()
            sys.exit(1)
        try:
                e = schedule.enter(0, 0, emit_robot_signals, (delay,  sig1,  ))
                schedule.run()
                loop.run()
        except (KeyboardInterrupt, dbus.DBusException, SystemExit):
                print "User requested exit... shutting down now"
                task_signal.Exit()
                local_signal.Exit()
                pass
                sys.exit(0)
