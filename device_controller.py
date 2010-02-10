import subprocess
import re
import time
import math
import sys
import  logging,  logging.config,  logging.handlers
#logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")


from myro import *
from RILCommonModules.RILSetup import *
from RILCommonModules.LiveGraph import *
from RILCommonModules.pose import *
from EpuckDistributedClient.data_manager import *
from EpuckDistributedClient.epuck_navigator import *

# Device status alias
DEVICE_NOT_RESPONDING = 0
DEVICE_AVAILABLE = 1
DEVICE_MOVING = 2
DEVICE_IDLE = 3
# globals
_packet_loss = ''
EXIT_COND = True 
# may be changed later to set as time duration 
SMALL_DELAY = 3
TASK_STEP_DURATION = 30

def process_ping_output(output):
    global _packet_loss
    pct = re.findall(r"\d\%", output)
    if(pct):
        #print "%loss: ", pct[0]
        _packet_loss = pct[0]
    if (_packet_loss == '0%'):
        #print "Device Alive"
        return True
    if (_packet_loss == '100%' or  (not _packet_loss)): 
        print "Device Dead"
        return False
    _packet_loss = ''

class DeviceController():
    def __init__(self,  dm,  bdaddr = 0,  task_period = TASK_STEP_DURATION):
        self.datamgr_proxy = dm
        self.robotid = dm.mRobotID
        self.pose = Pose()
        self.bdaddr = bdaddr
        self.task_start = 0 # time()
        self.task_end = 0 # time()
        self.task_period = task_period
        # shared status
        #self.l2ping_ok = False
        self.task_selected = False
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
    
    def InitLogFiles(self):
        # -- Init Stimuli writer --
        name = "RobotPose"
        now = time.strftime("%Y%b%d-%H%M%S", time.gmtime())
        desc = "logged in centralized communication mode from: " + now
        # prepare label
        label = "TimeStamp;StepCounter;X;Y;Theta \n"
        # Data context
        ctx = DataCtx(name, label, desc)
        self.pose_writer = DataWriter("Robot", ctx, now, str(self.robotid))

    def GetCommonHeader(self):
        ts = time.strftime("%H:%M:%S", time.gmtime())
        sep = DATA_SEP
        self.step = self.step + 1
        header = str(ts) + sep + str(self.step)
        return header
    
    def AppendPoseLog(self):        
        sep = DATA_SEP
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        p = self.pose
        log = self.GetCommonHeader()\
         + sep + str(p.x) + sep + str(p.y) + sep + str(p.theta) + "\n"
        try: 
            self.pose_writer.AppendData(log)
        except:
            print "Pose logging failed"

    def InitEpuck(self):
        self.epuck_ready = False
        try:
            self.epuck = Epuck(self.datamgr_proxy.mRobotID)
            self.epuck_ready = True
            #self.epuck.stop()
        except Exception, e:
            self.epuck_ready = False
            print "Failed to initialize Myro epuck robot:", e

    def EpuckReady(self):        
        return self.epuck_ready

    def TaskSelected(self):
        self.task_selected = False
        try:
            #self.datamgr_proxy.mSelectedTaskAvailable.wait()
            taskdict = self.datamgr_proxy.mSelectedTask
            taskid = eval(str(taskdict[SELECTED_TASK_ID]))
            status = str(taskdict[SELECTED_TASK_STATUS])
            print "From TaskDict got %i %s"  %(taskid,  status)
            #self.datamgr_proxy.mSelectedTaskAvailable.clear() # read complete
            if status == TASK_SELECTED:
                self.task_selected = True
            if taskid is RANDOM_WALK_TASK_ID:
                self.task_is_rw = True
        except Exception, e:
            logger.warn("@TaskSelected not available: %s", e)
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
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        if int(self.last_pose_ts) < int(self.pose.ts):
            updated = True
        return updated

    def GetUpdatedRobotPose(self):
        self.pose.UpdateFromList(self.datamgr_proxy.mRobotPose)
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.theta
        ts = self.pose.ts
        self.last_pose_ts = ts # used later, see PoseUpdated()
        return x, y, theta

    def GetTaskPoseXY(self):
        st = self.datamgr_proxy.mSelectedTask[SELECTED_TASK_INFO]
        x = st[TASK_INFO_X]
        y= st[TASK_INFO_Y]
        return x, y

    def DoRandomWalk(self):
        maxtime = time.time() + self.task_period
        try:
            logger.debug ("\tRandom walking")
            self.navigator.GoForward(self.epuck, maxtime)
            #time.sleep(self.task_period)
            self.navigator.AppendMotionLog()
        except Exception, e:
            logger.debug ("\tRandom walk failed %s", e)
    
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

    def ResetTaskStats(self):
        self.task_done = True
        self.task_end = time.time()
        self.datamgr_proxy.mTaskTimedOut.set() # Trigger TaskSelector, Emitter
        #time.sleep(1)
        #self.datamgr_proxy.mTaskTimedOut.clear() 
        self.task_is_rw = False
        self.task_timedout = False 
        self.task_selected = False

 
    def RunDeviceUnavailableLoop(self):
        while self.status is DEVICE_NOT_RESPONDING:
            logger.debug ("Entering DEVICE_NOT_RESPONDING loop--->")
            if self.EpuckReady():
                self.status = DEVICE_AVAILABLE # go out loop
                break
            else: 
                self.status = DEVICE_NOT_RESPONDING # stay here in loop
                time.sleep(SMALL_DELAY)

    def RunDeviceAvailableLoop(self):
        while self.status is DEVICE_AVAILABLE:
            logger.debug ("Entering DEVICE_AVAILABLE loop--->")
            self.DoRandomWalk() ## To find task
            if self.TaskSelected():
                self.task_start = time.time()
                self.status = DEVICE_MOVING  # go out of this loop
                self.datamgr_proxy.mSelectedTaskStarted.set() #>> dbus_emitter
                break
            if (not self.EpuckReady()):
                self.status = DEVICE_NOT_RESPONDING # go out of this loop
                break
            else:
               self.status = DEVICE_AVAILABLE # stay in-loop
               print "@DEVICE_AVAILABLE loop"
               self.DoRandomWalk() ## To improve id-pose detection
               #time.sleep(SMALL_DELAY)
   
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
                    self.MoveToTarget()
            else:
                self.status = DEVICE_MOVING # stay in-loop
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
                        time.sleep(SMALL_DELAY)
                    else:
                        self.status = DEVICE_MOVING # go out of loop
                        logger.debug ("\t go out to DEVICE_MOVING")
                        break
                else:
                    logger.debug ("\t Pose not Updated")
                    # stay in loop
                    self.status = DEVICE_IDLE 
                    # but do random walk to be seen by camera
                    self.DoRandomWalk()
            logger.debug ("--->Exiting DEVICE_IDLE loop.")              


    def RunMainLoop(self):          
        while EXIT_COND:
            try:
                self.RunDeviceUnavailableLoop()
                self.RunDeviceAvailableLoop()
                self.RunDeviceMovingLoop()
                self.RunDeviceIdleLoop()                
                time.sleep(1)
            except (KeyboardInterrupt, SystemExit):
                print "User requested exit..DeviceController shutting down now"                
                sys.exit(0)

# CODE++: parse based on robot-id
def get_config(config_file,  config):
    result = ' '
    f = open(config_file, 'r')
    data = f.read()
    lst = re.split(';', data)
    if (config == 'bdaddr'):
        result = lst[1]
    return result

def controller_main(data_mgr):
        #bdaddr = get_config(config_file,  'bdaddr')
        dc = DeviceController(data_mgr)
        dc.InitLogFiles()
        dc.InitEpuck()
        dc.RunMainLoop()

