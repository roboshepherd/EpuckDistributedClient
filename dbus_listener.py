import time, os, sys, sched, subprocess, re, signal, traceback
import gobject
import dbus, dbus.service, dbus.mainloop.glib 
import multiprocessing,  logging,  logging.config,  logging.handlers

from RILCommonModules.RILSetup import *
from RILCommonModules.LiveGraph import *
from RILCommonModules.utils import *
from EpuckDistributedClient.data_manager import *
from EpuckDistributedClient.utils import *

#logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")

#---------------------Log recevd. signal/data  ---------------------
class CommLogger():
    def __init__(self, dm):
        self.datamgr_proxy = dm
        self.robotid = dm.GetRobotID()
        self.log_writer1 = None  # for logging recvd. taskinfo signal
        self.log_writer2 = None  # for logging recvd. robotpeers signal            
        self.step = 0

    def InitLogFiles(self):
        name = "DBusListener-TaskSignal"
        now = time.strftime("%Y%b%d-%H%M%S", time.gmtime())
        desc = "logged in local communication mode from: " + now
        # prepare label
        label = "TimeStamp;HH:MM:SS;StepCounter;SRC; TaskInfoLen;TaskIDList \n"
        # Data context
        ctx = DataCtx(name, label, desc)
        # Signal Logger
        self.log_writer1 = DataWriter("Robot", ctx, now, str(self.robotid))
        name = "DBusListener-PeerSignal"
        label = "TimeStamp;HH:MM:SS;StepCounter;SRC; Len;Desc \n"
        # Data context
        ctx = DataCtx(name, label, desc)
        # Signal Logger
        self.log_writer2 = DataWriter("Robot", ctx, now, str(self.robotid))

    def _GetCommonHeader(self):
        sep = DATA_SEP
        ts = str(time.time()) + sep + time.strftime("%H:%M:%S", time.gmtime())
        self.step = self.step + 1
        header = ts + sep + str(self.step)
        return header
    
    def AppendCommLog(self, desc, taskinfo):        
        sep = DATA_SEP
        length = len(taskinfo)
        task_ids = taskinfo.keys()
        task_ids.sort() 
        log = self._GetCommonHeader() + sep + desc\
         + sep + str(length) + sep + str(task_ids) + "\n"
        try: 
            self.log_writer1.AppendData(log)
        except:
            print "TaskInfo signal logging failed"

    def AppendPeerLog(self, desc, peers):        
        sep = DATA_SEP
        length = len(peers)        
        peers.sort() 
        log = self._GetCommonHeader() + sep + desc\
         + sep + str(length) + sep + str(peers) + "\n"
        try: 
            self.log_writer2.AppendData(log)
        except:
            print "RobotPeers signal logging failed"


#--------------------- Signal Reception ----------------------------

#def save_pose_from_dbus_server(pose): # for emulated dbus_server
    #global datamgr_proxy
    #try:
        #datamgr_proxy.mRobotPose.clear()
        #for k, v in pose.iteritems():
            #key = str(k)
            #value = eval(str(v))
            #datamgr_proxy.mRobotPose[key] = value 
        #datamgr_proxy.mRobotPoseAvailable.set()
        #print datamgr_proxy.mRobotPose
        ##logger.info("RobotPose-x %f",  datamgr_proxy.mRobotPose[1])
        #logger.info("@DBC RobotPose recvd. len logged: %d" ,\
            #len(datamgr_proxy.mRobotPose))
    #except:
       #print "Err in save_pose()"

def swistrack_pose_signal_handler( x, y, theta):
    global datamgr_proxy
    #print "Caught pose signal: %f, %f, %f "  %(x, y, theta)
    try:
        datamgr_proxy.mRobotPose.clear()
        datamgr_proxy.mRobotPose[ROBOT_POSE_X] = eval(str(x))
        datamgr_proxy.mRobotPose[ROBOT_POSE_Y] = eval(str(y))
        datamgr_proxy.mRobotPose[ROBOT_POSE_THETA] = eval(str(theta))
        datamgr_proxy.mRobotPose[ROBOT_POSE_TS] = time.time()
        if (not datamgr_proxy.mRobotPoseAvailable.is_set()):
            datamgr_proxy.mRobotPoseAvailable.set()
    except:
       print "Err in save_pose()"

def taskserver_signal_handler(sig,  taskinfo):
    global datamgr_proxy
    #print "Caught signal  %s (in taskinfo signal handler) "  %(sig)
    #print "Val: ",  taskinfo
    comm_logger.AppendCommLog(" taskinfo (from taskserver)", taskinfo)	
    try:
        datamgr_proxy.mTaskInfo.clear()
        for k, v in taskinfo.iteritems():
            key = eval(str(k))
            value = extract_objects(v)
            datamgr_proxy.mTaskInfo[key] = value
        if (not datamgr_proxy.mTaskInfoAvailable.is_set()):
            datamgr_proxy.mTaskInfoAvailable.set()
        #print datamgr_proxy.mTaskInfo
    except:
       print "Err in save_taskinfo()"

def swistrack_peers_signal_handler(robotid, val):
    global datamgr_proxy
    #print "Caught signal over /robot%s (robot peers signal handler)" %(robotid)
    #print "Val: ",  val    
    peers = extract_objects(val)
    #print "Got peers:", peers
    comm_logger.AppendPeerLog(" peerlist (from swistrack)", peers)
    datamgr_proxy.mRobotPeers[TIME_STAMP] = time.time()
    datamgr_proxy.mRobotPeers[ROBOT_PEERS] = peers
    if (not datamgr_proxy.mRobotPeersAvailable.is_set()):
            datamgr_proxy.mRobotPeersAvailable.set()

def local_taskinfo_signal_handler(robotid, taskinfo):
    global datamgr_proxy
    logger.debug("Caught local taskinfo signal for robot%s\
     (local signal handler)", robotid)    
    if (int(robotid)) == int(datamgr_proxy.mRobotID):
        logger.debug("Taskinfo sent to me: %s",  taskinfo)
        comm_logger.AppendCommLog(" taskinfo (from peer)", taskinfo)
    else:
        return
    # TODO: put taskinfo of peers into data manger
    try:
        local_taskinfo = {}
        local_taskinfo = datamgr_proxy.mLocalTaskInfo.copy()
        for k, v in taskinfo.iteritems():
            key = eval(str(k))
            taskinfo_rcvd = extract_objects(v)
            if key in local_taskinfo.keys():
                taskinfo_old = local_taskinfo[key]
                if (int(taskinfo_rcvd[TASK_INFO_TIME])\
                  > int(taskinfo_old[TASK_INFO_TIME])):                    
                    datamgr_proxy.mLocalTaskInfo[key] = taskinfo_rcvd
                    print "Local task info updated @dm now:"
                    #print datamgr_proxy.mLocalTaskInfo
                else:
                    print "Robot's local  info is more recent:"
                    #print "@dm now:"
                    #print datamgr_proxy.mLocalTaskInfo
            else: # old dict was empty
                datamgr_proxy.mLocalTaskInfo[key] = taskinfo_rcvd
                print "Local task info added @dm now:"
                #print  datamgr_proxy.mLocalTaskInfo
                if (not datamgr_proxy.mTaskInfoAvailable.is_set()):
                    datamgr_proxy.mTaskInfoAvailable.set()
    except Exception, e:
        print "Err:", e
        
def main_loop():
    global loop
    try:
        loop = gobject.MainLoop()
        loop.run()
    except (KeyboardInterrupt, SystemExit):
        print "User requested exit... shutting down now"
        sys.exit(0)

def listener_main(data_mgr,  dbus_if1= DBUS_IFACE_TRACKER,\
            dbus_path1 = DBUS_ROBOT_PATH_BASE,\
            dbus_if2= DBUS_IFACE_TASK_SERVER, \
            dbus_path2 = DBUS_PATH_TASK_SERVER,\
            dbus_if4 = DBUS_IFACE_EPUCK,\
            robots_cfg = ROBOTS_PATH_CFG_FILE,\
            sig1 = SIG_ROBOT_POSE, sig2= SIG_TASK_INFO, sig3= SIG_ROBOT_PEERS,\
            sig4= SIG_LOCAL_TASK_INFO, delay=3 ):
        print "Initializing dbus listener"
        global datamgr_proxy,  task_signal, comm_logger 
        datamgr_proxy = data_mgr
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        #print "%s, %s, %s" %(dbus_if1, dbus_path1, sig1)
        dbus_path3 = dbus_path1 + str(data_mgr.mRobotID)
	# setup logging signals        
        comm_logger = CommLogger(data_mgr)
        comm_logger.InitLogFiles()
        try:
            # catch swistrack's pose_signal
            bus.add_signal_receiver(swistrack_pose_signal_handler,\
             dbus_interface = dbus_if1, path= dbus_path3,  signal_name = sig1)
            # catch swistrack's robot peers signal
            bus.add_signal_receiver(swistrack_peers_signal_handler,\
             dbus_interface= dbus_if1, path = dbus_path3,  signal_name = sig3) 
            # catch task server's real task info
            bus.add_signal_receiver(taskserver_signal_handler, dbus_interface\
             = dbus_if2, path= dbus_path3,  signal_name = sig2)
            # catch peer's local task info
            local_path = dbus_path3 + 'local'
            bus.add_signal_receiver(local_taskinfo_signal_handler,\
             dbus_interface= dbus_if4, path = local_path, signal_name = sig4)
            main_loop()
        except dbus.DBusException:
            traceback.print_exc()
            sys.exit(1)
