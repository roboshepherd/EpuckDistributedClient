#!/usr/bin/env python
import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging

from RILCommonModules.RILSetup import *
from EpuckDistributedClient.utils import *
from EpuckDistributedClient.data_manager import *


schedule1 = sched.scheduler(time.time, time.sleep)
schedule2 = sched.scheduler(time.time, time.sleep)
loop = None

#------------------ Signal Despatch ---------------------------------
class RobotTaskEngagementSignal(dbus.service.Object):
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

class LocalTaskInfoSignal(dbus.service.Object):
    def __init__(self, object_path):
        dbus.service.Object.__init__(self, dbus.SessionBus(), object_path)
    @dbus.service.signal(dbus_interface= DBUS_IFACE_EPUCK,\
     signature='ia{iad}')
    def  LocalTaskInfo(self,  robotid,  taskinfo):
        #logger.info("Emitted %s : Robot %d now %s ",  sig,  taskid,  status)
        #print "LocalTaskInfo emitted for robot%i : local taskinfo: %s"\
        # %(robotid,  taskinfo)
        pass
    def Exit(self):
        global loop
        loop.quit()

def  emit_local_taskinfo_signal(delay, sig):
    global datamgr_proxy, local_signal
    # reschedule
    print "Entering to emit_local_taskinfo_signal ..."
    schedule1.enter(delay+1, 0, emit_local_taskinfo_signal, (delay+1, sig  ) )
    # emit robot's local taskinfo
    try:
        datamgr_proxy.mRobotPeersAvailable.wait()
        #datamgr_proxy.mTaskInfoAvailable.wait()
        robotid = datamgr_proxy.mRobotID
        taskinfo = datamgr_proxy.mLocalTaskInfo.copy()
        peers = datamgr_proxy.mRobotPeers[ROBOT_PEERS]
        for peerid in peers:
            print "Emitting target peer channel: /robot", peerid
            if int(peerid) > robotid:
                local_signal[int(peerid) - 2].LocalTaskInfo(peerid,  taskinfo)
            else:
                local_signal[int(peerid) - 1].LocalTaskInfo(peerid,  taskinfo)
        if datamgr_proxy.mRobotPeersAvailable.is_set():
            datamgr_proxy.mRobotPeersAvailable.clear()
    except Exception, e:
        print "Err at emit_robot_status_signal():", e

def emit_robot_status_signal(delay,  sig1):
    global task_signal,  datamgr_proxy
    # reschedule 
    schedule1.enter(delay, 0, emit_robot_status_signal, (delay, sig1  ) )
    ## emit robot's task activity signal
    try:
        datamgr_proxy.mSelectedTaskStarted.wait() ### Blocking !!! ###
        robotid = datamgr_proxy.mRobotID
        taskdict = datamgr_proxy.mSelectedTask
        datamgr_proxy.mSelectedTaskStarted.clear()
        taskid =  eval(str(taskdict[SELECTED_TASK_ID])) 
        status = str(taskdict[SELECTED_TASK_STATUS]) 
        #print "From TaskDict got %i %s"  %(taskid,  status)
        task_signal.TaskStatus(sig1,  robotid,  taskid)
        ### ------------------- NEED TO SEE THE EFFECT -----------------##
        if datamgr_proxy.mTaskTimedOut.is_set():
            datamgr_proxy.mSelectedTaskStarted.clear()
        ### -------------------------------------------------------------##
    except:
        print "Emitting Robot Task Status signal failed"
   

def emitter_main(dm,  dbus_iface= DBUS_IFACE_EPUCK,\
    dbus_path1 = DBUS_PATH_BASE,\
    sig1 = SIG_TASK_STATUS, sig2 = SIG_LOCAL_TASK_INFO,\
    robots_cfg = ROBOTS_PATH_CFG_FILE, delay = 5):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        session_bus = dbus.SessionBus()
        global task_signal,  datamgr_proxy, local_signal
        datamgr_proxy = dm
        status_path = dbus_path1 + str(dm.mRobotID)
        dbus_paths = GetDBusPaths(robots_cfg)
        tmp_paths = []
        for x in dbus_paths:
            if x != dbus_path1: # exclude self path to bounce msg to self
                tmp_paths.append(x)
        local_paths = [(x + 'local') for x in tmp_paths]
        try:
            name = dbus.service.BusName(dbus_iface, session_bus)
            task_signal = RobotTaskEngagementSignal(status_path)
            local_signal = []
            for p in local_paths:
                local_signal.append(LocalTaskInfoSignal(p))
            loop = gobject.MainLoop()
            print "Running Outbound Robot Signalling services."
        except dbus.DBusException:
            traceback.print_exc()
            sys.exit(1)
        try:
                e1 = schedule1.enter(1, 0, emit_robot_status_signal,\
                 (delay,  sig1,  ))
                e2 = schedule1.enter(2, 0, emit_local_taskinfo_signal,\
                 (delay+1,  sig2,  )) 
                schedule1.run()
                #schedule2.run()
                loop.run()
        except (KeyboardInterrupt, dbus.DBusException, SystemExit):
                print "User requested exit... shutting down now"
                task_signal.Exit()
                local_signal.Exit()
                sys.exit(0)
