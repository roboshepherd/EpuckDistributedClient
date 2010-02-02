import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging,  logging.config,  logging.handlers

from RILCommonModules.RILSetup import *
from RILCommonModules.utils import *
from EpuckDistributedClient.data_manager import *

#logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")

#--------------------- Signal Reception ----------------------------

def save_pose_from_dbus_server(pose):
    global datamgr_proxy
    try:
        datamgr_proxy.mRobotPose.clear()
        for k, v in pose.iteritems():
            key = str(k)
            value = eval(str(v))
            datamgr_proxy.mRobotPose[key] = value 
        datamgr_proxy.mRobotPoseAvailable.set()
        print datamgr_proxy.mRobotPose
        #logger.info("RobotPose-x %f",  datamgr_proxy.mRobotPose[1])
        logger.info("@DBC RobotPose recvd. len logged: %d" ,\
            len(datamgr_proxy.mRobotPose))
    except:
       print "Err in save_pose()"

def pose_signal_handler( x, y, theta):
    global datamgr_proxy
    #print "Caught pose signal: %f, %f, %f "  %(x, y, theta)
    try:
        datamgr_proxy.mRobotPose.clear()
        datamgr_proxy.mRobotPose[ROBOT_POSE_X] = eval(str(x))
        datamgr_proxy.mRobotPose[ROBOT_POSE_Y] = eval(str(y))
        datamgr_proxy.mRobotPose[ROBOT_POSE_THETA] = eval(str(theta))
        datamgr_proxy.mRobotPose[ROBOT_POSE_TS] = time.time()
        datamgr_proxy.mRobotPoseAvailable.set()
    except:
       print "Err in save_pose()"

def taskinfo_signal_handler(sig,  taskinfo):
    global datamgr_proxy
    #print "Caught signal  %s (in taskinfo signal handler) "  %(sig)
    #print "Val: ",  val
    try:
        datamgr_proxy.mTaskInfo.clear()
        for k, v in taskinfo.iteritems():
            key = eval(str(k))
            value = extract_objects(v)
            datamgr_proxy.mTaskInfo[key] = value 
        datamgr_proxy.mTaskInfoAvailable.set()
        #print datamgr_proxy.mTaskInfo
    except:
       print "Err in save_taskinfo()"

def robot_peers_signal_handler(sig, val):
    global datamgr_proxy
    print "Caught signal  %s (in taskinfo signal handler) "  %(sig)
    print "Val: ",  val
 
def main_loop():
    global loop
    try:
        loop = gobject.MainLoop()
        loop.run()
    except (KeyboardInterrupt, SystemExit):
        print "User requested exit... shutting down now"
        sys.exit(0)

def listener_main(data_mgr,  dbus_if1= DBUS_IFACE_TRACKER,\
            dbus_path1 = DBUS_PATH_BASE,\
            dbus_if2= DBUS_IFACE_TASK_SERVER, \
            dbus_path2 = DBUS_PATH_TASK_SERVER,\
            sig1 = SIG_ROBOT_POSE, sig2 = SIG_TASK_INFO, sig3= SIG_ROBOT_PEERS\
            delay=3 ):
        print "Initializing dbus listener"
        global datamgr_proxy,  task_signal
        datamgr_proxy = data_mgr
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        #print "%s, %s, %s" %(dbus_if1, dbus_path1, sig1)
        dbus_path3 = "/robot" + str(data_mgr.mRobotID)
        try:
            bus.add_signal_receiver(pose_signal_handler, dbus_interface =\
                dbus_if1, path= dbus_path1,  signal_name = sig1)
            bus.add_signal_receiver(taskinfo_signal_handler, dbus_interface =\
                dbus_if2, path= dbus_path2,  signal_name = sig2)
            bus.add_signal_receiver(robot_peers_signal_handler, dbus_interface\
                = dbus_if1, path = dbus_path3,  signal_name = sig3)        
            main_loop()
        except dbus.DBusException:
            traceback.print_exc()
            sys.exit(1)
