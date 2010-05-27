import subprocess
import re
import time
import math
import sys
import  logging,  logging.config,  logging.handlers
logger = logging.getLogger("EpcLogger")
from myro import *
from RILCommonModules.RILSetup import *
from RILCommonModules.LiveGraph import *
from RILCommonModules.pose import *
from EpuckDistributedClient.data_manager import *
from EpuckDistributedClient.epuck_navigator import *

# BT link setup/reset
BT_CONF_FILE =\
 '/home/newport-ril/centralized-expt/btconfig/hci-bdaddr-remote.conf'

# Device status alias
DEVICE_NOT_RESPONDING = 0
DEVICE_AVAILABLE = 1
DEVICE_MOVING = 2
DEVICE_IDLE = 3
STATUS_SCALE = 1000 # for better visualization
TASK_SCALE = 10
# globals
_packet_loss = ''
EXIT_COND = True 
# may be changed later to set as time duration 
SMALL_DELAY = 3 # orig : 3
LONG_TIME_X = 5 # multiplier of SMALL_DELAY
TASK_STEP_DURATION = 30

class DeviceController():
    def __init__(self,  dm, task_period = TASK_STEP_DURATION):
        self.datamgr_proxy = dm
        self.robotid = dm.GetRobotID()
        self.pose = Pose()
        self.task_start = 0 # time()
        self.task_end = 0 # time()
        self.task_period = task_period
        # shared status
        #self.l2ping_ok = False
        self.task_selected = False
        self.target_task = None
        self.task_is_rw = False
        self.task_started = False
        self.task_pending = False
        self.task_done = False
        self.task_timedout = False
        self.at_task = False
        #self.pose_available = False
        self.last_pose_ts = 0 # last time stamp
        # Device status
        self.status = DEVICE_NOT_RESPONDING
        # myro robot
        self.epuck = None
        self.epuck_ready = False
        # navigator
        self.navigator = EpuckNavigator()
        # data logger
        self.step = 0
        self.pose_writer = None
        self.status_writer = None
        # BT link setup
        self.bt_up = False        
        self.bdaddr = None
        self.hciID = None
        self.dev_rfcomm = None
    
    def InitLogFiles(self):
        # -- Init Stimuli writer --
        name = "RobotPose"
        now = time.strftime("%Y%b%d-%H%M%S", time.gmtime())
        desc = "logged in centralized communication mode from: " + now
        # prepare label
        label = "TimeStamp;HH:MM:SS;StepCounter;X;Y;Theta \n"
        # Data context
        ctx = DataCtx(name, label, desc)
        self.pose_writer = DataWriter("Robot", ctx, now, str(self.robotid))
        # status writer
        name = "DeviceStatus"
        label = "TimeStamp;HH:MM:SS;StepCounter;DeviceStatus \n"
        ctx = DataCtx(name, label, desc)
        self.status_writer = DataWriter("Robot", ctx, now, str(self.robotid))

    def GetCommonHeader(self):
        sep = DATA_SEP
        ts = str(time.time()) + sep + time.strftime("%H:%M:%S", time.gmtime())
        self.step = self.step + 1
        header = ts + sep + str(self.step)
        return header
    
    def AppendPoseLog(self):        
        sep = DATA_SEP
        self.pose.UpdateFromList(self.datamgr_proxy.GetRobotPose())
        p = self.pose
        log = self.GetCommonHeader()\
         + sep + str(p.x) + sep + str(p.y) + sep + str(p.theta) + "\n"
        try: 
            self.pose_writer.AppendData(log)
        except:
            print "Pose logging failed"
    
    def AppendStatusLog(self, status):        
        sep = DATA_SEP
        log = self.GetCommonHeader()\
         + sep + str(self.step) + sep + str(status) + "\n"
        try: 
            self.status_writer.AppendData(log)
        except:
            print "Status logging failed"
            
    def ResetBTLink(self):
        print "BT Resetting..."
        logger.debug("BT Resetting...")
        f = open(BT_CONF_FILE, 'r')
        for line in f.readlines():
            if (line.split("/")[0] == str(self.robotid)):
                self.hciID = line.split("/")[1]
                self.bdaddr = line.split("/")[2]                
        self.dev_rfcomm = "/dev/rfcomm" + str(self.robotid)
        # release dev
        if self.bt_up:
            cmd = 'rfcomm release ' + self.dev_rfcomm
            print ">>>>>> Doing: ", cmd
            subproc = subprocess.Popen([cmd, ], stderr=subprocess.PIPE,\
             stdout=subprocess.PIPE, shell=True)
            stdout_value = subproc.communicate()
            print stdout_value[0]
            logger.debug("Releasing rfcomm dev: %s",stdout_value[0])            
        # bind dev
        time.sleep(SMALL_DELAY)        
        cmd = 'rfcomm ' + ' -i ' + self.hciID + ' bind ' + self.dev_rfcomm\
         + ' ' + self.bdaddr + '  1'  # channel is always 1
        print ">>>>>> Doing: ", cmd
        subproc = subprocess.Popen([cmd, ], stderr=subprocess.PIPE,\
         stdout=subprocess.PIPE, shell=True)
        stdout_value = subproc.communicate()
        print stdout_value[0]
        logger.debug("Binding rfcomm dev: %s",stdout_value[0])
        # set status
        self.bt_up = True

    def InitEpuck(self):
        self.epuck_ready = False
        try:
            self.epuck = Epuck(self.datamgr_proxy.GetRobotID())
            self.epuck_ready = True
            #self.epuck.stop()
        except Exception, e:
            self.epuck_ready = False
            log = "Failed to initialize Myro epuck robot:" + str(e)
            print log
            logger.warn(log)
    def EpuckReady(self):
        if (not self.epuck_ready):
            self.InitEpuck()
        return self.epuck_ready

    def TaskSelected(self):
        self.task_selected = False
        try:
            #self.datamgr_proxy.WaitSelectedTaskAvailable()
            taskdict = eval(self.datamgr_proxy.GetSelectedTask())
            #print type(td)
            print taskdict[SELECTED_TASK_ID]
            taskid = int(taskdict[SELECTED_TASK_ID])
            status = taskdict[SELECTED_TASK_STATUS]
            self.target_task = taskid
            log = "From TaskDict got %i %s"  %(taskid,  status)
            print log
            logger.info(log)
            #self.datamgr_proxy.ClearSelectedTaskAvailable() # read complete        
            if status == TASK_SELECTED:
                self.task_selected = True
            if taskid is RANDOM_WALK_TASK_ID:
                self.task_is_rw = True
        except Exception, e:
            log = "@DC Task Selection not available:" + str(e)
            print log
            logger.warn(log)
        return self.task_selected
    
    def RandomWalkTask(self):
        return self.task_is_rw 

    def TaskTimedOut(self):
        self.task_timedout = False
        now = time.time()
        elasped = now - self.task_start
        if elasped > self.task_period:
            self.task_timedout = True
        return self.task_timedout

    def ArrivedAtTask(self):
        self.at_task = False
        robot_x, robot_y, robot_theta = self.GetUpdatedRobotPose()
        task_x, task_y = self.GetTaskPoseXY()
        dx = math.fabs(robot_x - task_x)
        dy = math.fabs(robot_y - task_y)
        #print "ArrivedAtTask(): xdist: %f ydist: %f" %(dx, dy)
        pxdist = math.sqrt(dx * dx + dy * dy)
        if (pxdist < TASK_RADIUS) :
            self.at_task = True
            #print "ArrivedAtTask(): Arrived within radius %.2f !\n" %pxdist
        return self.at_task
    
    def PoseUpdated(self):
        updated = False
        pl = eval(self.datamgr_proxy.GetRobotPose())
        self.pose.UpdateFromList(pl)
        if int(self.last_pose_ts) < int(self.pose.ts):
            updated = True
        return updated

    def GetUpdatedRobotPose(self):
        p = eval(self.datamgr_proxy.GetRobotPose())
        self.pose.UpdateFromList(p)
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.theta
        ts = self.pose.ts
        self.last_pose_ts = ts # used later, see PoseUpdated()
        return x, y, theta

    def GetTaskPoseXY(self):
        x = 0
        y = 0 
        try:
            td = eval(self.datamgr_proxy.GetSelectedTask())
            logger.warn("GetTaskPoseXY(): selected task %s", td)
            st = td[SELECTED_TASK_INFO]
            x = st[TASK_INFO_X]
            y= st[TASK_INFO_Y]
        except Exception, e:
            log = "GetTaskPoseXY() Er: " + str(e)
            print log
            logger.warn(log)
        return x, y

    def DoRandomWalk(self):
        maxtime = time.time() + self.task_period
        try:
            logger.debug ("\tRandom walking")
            self.navigator.GoForward(self.epuck, maxtime)
            
            #time.sleep(self.task_period)
            self.navigator.AppendMotionLog()
        except Exception, e:
            logger.debug ("\tRandom walk failed %s", str(e))
            self.epuck_ready = False
    
    def MoveToTarget(self):
        rx, ry, rtheta = self.GetUpdatedRobotPose()
        task_x, task_y = self.GetTaskPoseXY()
        if self.ArrivedAtTask():
            logger.info("****** Arrived Task Location *******")
            return
        maxtime = time.time() + self.task_period
        try:
            logger.debug ("\t--> Naviagtor")
            self.navigator.GoTowardsTarget(self.epuck,\
             rx, ry, rtheta, task_x, task_y, maxtime)
            #time.sleep(self.task_period)
            self.navigator.AppendMotionLog()
        except Exception,e:
            logger.warn("Going towards target failed: %s", e)
            self.epuck_ready = False

    def ResetTaskStats(self):
        self.task_done = True
        self.task_end = time.time()
        self.datamgr_proxy.SetTaskTimedOut() # Trigger TaskSelector
        #time.sleep(1)
        #self.datamgr_proxy.mTaskTimedOut.clear() 
        self.task_is_rw = False
        self.task_timedout = False 
        self.task_selected = False
        self.target_task = None

    def RunDeviceUnavailableLoop(self):
        while self.status is DEVICE_NOT_RESPONDING:
            logger.debug ("Entering DEVICE_NOT_RESPONDING loop--->")
            self.ResetBTLink()
            if self.EpuckReady():
                self.status = DEVICE_AVAILABLE # go out loop
                break
            else: 
                self.status = DEVICE_NOT_RESPONDING # stay here in loop
                self.AppendStatusLog(str(DEVICE_NOT_RESPONDING))
                time.sleep(LONG_TIME_X*SMALL_DELAY)

    def RunDeviceAvailableLoop(self):
        while self.status is DEVICE_AVAILABLE:
            logger.debug ("Entering DEVICE_AVAILABLE loop--->")
            self.AppendStatusLog(str(DEVICE_AVAILABLE * STATUS_SCALE))
            self.DoRandomWalk() ## To find task
            if self.TaskSelected():
                self.task_start = time.time()
                self.status = DEVICE_MOVING  # go out of this loop
                self.datamgr_proxy.SetSelectedTaskStarted() #>> dbus_emitter
                break
            if (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING # go out of this loop
                break
            else:
               self.status = DEVICE_AVAILABLE # stay in-loop
               print "@DEVICE_AVAILABLE loop"
               self.AppendStatusLog(str(DEVICE_AVAILABLE * STATUS_SCALE))
               self.DoRandomWalk() ## To improve id-pose detection
               time.sleep(SMALL_DELAY)
   
    def RunDeviceMovingLoop(self):
        while self.status is DEVICE_MOVING:
            logger.debug ("Entering DEVICE_MOVING loop--->")
            if (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                logger.debug ("\t DEVICE_NOT_RESPONDING")
                break
            elif self.TaskTimedOut():
                self.status = DEVICE_AVAILABLE # go out of loop
                logger.debug ("\t Timedout")
                self.ResetTaskStats() # for next task
                break 
            # ---------------- Doing task ----------------------#
            if  self.task_is_rw:
                self.status = DEVICE_MOVING # stay in-loop
                # do random walking ...
                logger.debug ("\t RandomWalking")
                self.AppendPoseLog()
                self.AppendStatusLog(str(DEVICE_MOVING * STATUS_SCALE) )
                self.DoRandomWalk()
            elif self.PoseUpdated(): # task is move to target
                if self.ArrivedAtTask():
                    logger.debug ("\t AtTask")
                    self.status = DEVICE_IDLE
                    break
                else:
                    logger.debug ("\tMoving to Task")
                    self.status = DEVICE_MOVING # stay in-loop
                    # go to navigation routines for MoveToTarget
                    self.AppendPoseLog()
                    self.AppendStatusLog(str(DEVICE_MOVING * STATUS_SCALE +\
                     self.target_task * TASK_SCALE))
                    self.MoveToTarget()
            else:
                self.status = DEVICE_MOVING # stay in-loop
                self.AppendStatusLog(str(DEVICE_MOVING * STATUS_SCALE))
                self.DoRandomWalk()
            logger.debug ("--->Exiting DEVICE_MOVING loop.")    
        
    def RunDeviceIdleLoop(self):
        while self.status is DEVICE_IDLE:
            logger.debug ("Entering DEVICE_IDLE loop--->")
            if not self.EpuckReady():
                self.status = DEVICE_NOT_RESPONDING # go out of loop
                logger.debug ("\t DEVICE_NOT_RESPONDING")
                break
            elif self.TaskTimedOut():
                self.status = DEVICE_AVAILABLE # go out of loop
                logger.debug ("\t Timedout")
                self.ResetTaskStats()
                break
            while (not self.TaskTimedOut()):
                if self.PoseUpdated():
                    if self.ArrivedAtTask():
                        logger.debug ("\t AtTask")
                        # stay in loop
                        self.status = DEVICE_IDLE
                        self.AppendStatusLog(str(DEVICE_IDLE * STATUS_SCALE +\
                         self.target_task * TASK_SCALE))
                        time.sleep(SMALL_DELAY)
                    else:
                        self.status = DEVICE_MOVING # go out of loop
                        logger.debug ("\t go out to DEVICE_MOVING")
                        break
                else:
                    logger.debug ("\t waiting for pose update")
                    # stay in loop
                    self.status = DEVICE_IDLE 
                    # but do random walk to be seen by camera
                    self.AppendStatusLog(str(DEVICE_MOVING * STATUS_SCALE))
                    self.DoRandomWalk()
            logger.debug ("--->Exiting DEVICE_IDLE loop.")              


    def RunMainLoop(self):          
        while EXIT_COND:
            try:
                self.RunDeviceUnavailableLoop()
                self.RunDeviceAvailableLoop()
                self.RunDeviceMovingLoop()
                self.RunDeviceIdleLoop()                
                time.sleep(0.1)
            except (KeyboardInterrupt, SystemExit):
                print "User requested exit..DeviceController shutting down now"                
                if self.epuck:
                    self.epuck.stop()
                sys.exit(0)



def controller_main(data_mgr):
        dc = DeviceController(data_mgr)
        dc.InitLogFiles()
        #dc.ResetBTLink()
        dc.InitEpuck()
        dc.RunMainLoop()

