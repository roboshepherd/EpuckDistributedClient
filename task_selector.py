import math
import  time
import  random
import gobject # !!
import sys
import logging,  logging.config,  logging.handlers

from RILCommonModules.RILSetup import *
from RILCommonModules.LiveGraph import *
from EpuckDistributedClient.data_manager import *
from EpuckDistributedClient.ril_robot import *

#logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")

LOG_TYPE_STIMULUS = "stimulus"
LOG_TYPE_DIST = "dist"
LOG_TYPE_SENSITIZATION = "sensitization"
LOG_TYPE_URGENCY = "urgency"
RW_TH = RW_MAX_URGENCY # Attempt 1: 0.5

class TaskProbRange():
    def __init__(self,  id):
        self.id = id
        self.start = 0
        self.end = 0

class TaskSelector():
    def __init__(self, dm,  robot): # taskinfo as recvd from dbus signal
        self.datamgr = dm
        self.robot =  robot # ril_robot
        self.taskinfo = dm.mLocalTaskInfo
        self.stimulus = []
        self.probabilities = []
        self.taskranges = [] # put inst. of TaskProbRange()s
        self.selected_taskid = -1 
        self.deltadist = DELTA_DISTANCE
        # live graph data writer objects
        self.step = 0
        self.data_ctx = None
        self.stimuli_writer = None
        self.dist_writer = None
        self.sensitization_writer = None
        self.pose_writer = None
        self.urgency_writer = None
        self.taskinfo_writer = None

    def  CalculateDist(self,  rp,  tx,  ty):
        if USE_NORMALIZED_POSE is True:
            x1 = rp.x/(MAX_X * POSE_FACTOR)
            y1 = rp.y/(MAX_Y * POSE_FACTOR)
            x2 = tx/(MAX_X * POSE_FACTOR)
            y2 = ty/(MAX_Y * POSE_FACTOR)
        else:
            x1 = rp.x
            y1 = rp.y
            x2 = tx
            y2 = ty
        return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

    def CalculateTaskStimuli(self,  learn, dist, deltadist,  urgency):
        stimuli = math.tanh(learn * urgency / ( dist + deltadist))
        return stimuli
    
    def CalculateRandomWalkStimuli(self,  taskstimulus,  taskcount):
        stimuli = math.tanh( 1 - (taskstimulus /  (taskcount + 1)))
        stimuli = math.fabs(stimuli)
        if (stimuli > RW_TH):
            stimuli = RW_TH
        return stimuli

    def CalculateProbabilities(self):
        r = self.robot
        dm = self.datamgr
        self.stimulus = []
        #logger.debug("@TS Robot pose %s:" , dm.mRobotPose.items() )
        r.pose.UpdateFromList(dm.mRobotPose)
        logger.debug("@TS  Robot pose x=%f y=%f:" , r.pose.x,  r.pose.y )
        ##--------------------TEST THIS  ---------------------------##
        ti = dm.mLocalTaskInfo.copy()
        logger.debug("listened taskinfo from peers: %s", ti)
        # Combine both local peer's and task-server's taskinfo, if any
        tmp = dm.mTaskInfo.copy()
        logger.debug("perceived taskinfo from server: %s", tmp)        
        try:
            ti.update(tmp)
            self.AppendTaskInfoLog(ti)            
            # update local taskinfo since this will be emitted to peers
            for k in ti.keys():
                dm.mLocalTaskInfo[k] = ti[k]
            logger.debug("Merged TaskInfo':%s", dm.mLocalTaskInfo.items())
        except Exception, e:
            print "Err at CalculateProbabilities():", e
        ##-----------------------------------------------------------##
        logger.debug("\t TaskInfo: %s",  ti.items() )
        taskCount = len(ti)
        logger.debug("\t task count %d:" , taskCount)
        try:
            for index,  info in ti.items():
                taskid = index 
                logger.debug("Taskid -- %i:" ,  taskid)
                tx,  ty = info[TASK_INFO_X],  info[TASK_INFO_Y]
                dist = self.CalculateDist(r.pose,  tx, ty)
                #logger.debug("\tTask dist %f:" ,  dist)
                learn =  r.taskrec[taskid].sensitization
                #logger.debug("\tTask learn %f:" ,  learn)
                urg = info[TASK_INFO_URGENCY ]
                #logger.debug("\tTask urg %f:" ,  urg)
                stimuli = self.CalculateTaskStimuli(learn, dist, self.deltadist, urg)
                logger.info("\tTask %d stimuli %f:" , taskid,  stimuli)
                self.stimulus.append(stimuli)
                # save claculation for logging and using in next step
                r.taskrec[taskid].id = taskid
                r.taskrec[taskid].sensitization = learn
                r.taskrec[taskid].dist = dist
                r.taskrec[taskid].stimuli = stimuli
                r.taskrec[taskid].urgency = urg
                # Forget about near-zero sensitized tasks
                if (learn <= 0):
                    del dm.mLocalTaskInfo[taskid]                
        except:
            logger.error("FIXME --  list error")
        tsSum = math.fsum(self.stimulus)
        #logger.debug("@TS Task Stimulus sum: %f",  tsSum)
        rwStimuli = self.CalculateRandomWalkStimuli(tsSum,  taskCount)
        logger.info("\t RandomWalk Stimuli: %f",  rwStimuli)
        taskid = 0
        r.taskrec[taskid].stimuli = rwStimuli
        stimulusSum = tsSum +  rwStimuli
        while taskid <= MAX_SHOPTASK:
            pb =  r.taskrec[taskid].stimuli / stimulusSum
            r.taskrec[taskid].probability = pb
            logger.debug("@TS Task %d Prob %f",  taskid,  pb )
            taskid = taskid + 1
    
    def ConvertProbbToRange(self):
        self.taskranges = []
        robot = self.robot
        tasks = len(robot.taskrec)
        startup = 0
        endsave = 0
        for taskid in range (tasks):
            pb = robot.taskrec[taskid].probability
            if (pb > 0):
                e = pb * PROB_SCALE
                end = int(round(e)) + endsave
            else:
                continue
            r =  TaskProbRange(taskid)
            r.start = startup
            r.end = end
            startup = end + 1
            endsave = end
            logger.debug("@TS Task %d prob start: %d  end: %d",  r.id, r.start,
            r.end )
            self.taskranges.append(r)
        return end
    
    def GetRandomSelection(self, end):
        ranges =  self.taskranges
        #logger.debug("@TS Task Count including RW: %d",  len(ranges))
        n = random.randrange(0, end, 1)
        logger.debug("\tSelected Randon no: %d",  n)
        for r in ranges:
            logger.debug("@TS Probing task %d range %d - %d ",  r.id, r.start,\
            r.end)
            if(n >= r.start and n <= r.end):
                logger.info("\t Select task %d ",  r.id)
                print "\t Select task %d "  %r.id
                self.selected_taskid = r.id
                break

    def PostTaskSelection(self):
        taskid = self.selected_taskid
        try:
            self.datamgr.mSelectedTask[SELECTED_TASK_ID] = taskid
            self.datamgr.mSelectedTask[SELECTED_TASK_STATUS] = TASK_SELECTED
            self.datamgr.mSelectedTask[SELECTED_TASK_INFO] =\
                self.datamgr.mTaskInfo[taskid]
        except Exception, e:
            logger.warn("Datamgr: SELECTED_TASK not updated ")
            print e

        self.robot.UpdateTaskRecords(self.selected_taskid)
        
        self.datamgr.mSelectedTaskAvailable.set() # Trigger Device Controller
        #time.sleep(1)
        if self.datamgr.mTaskTimedOut.is_set():
            self.datamgr.mTaskTimedOut.clear() # delay next task selection
        if self.datamgr.mRobotPoseAvailable.is_set():
            self.datamgr.mRobotPoseAvailable.clear()
        if self.datamgr.mTaskInfoAvailable.is_set():
            self.datamgr.mTaskInfoAvailable.clear()


    def InitLogFiles(self):
        # -- Init Stimuli writer --
        name = "Stimulus"
        now = time.strftime("%Y%b%d-%H%M%S", time.gmtime()) 
        desc = "logged in local communication mode from: " + now +"\n"
        # prepare label
        label = "TimeStamp;HH:MM:SS;StepCounter;SelectedTask"
        for x in xrange(0,  MAX_SHOPTASK+1):
            label += "; "
            label += "Task"
            label += str(x)
        label += "\n"
        # Data context
        self.data_ctx = DataCtx(name, label, desc)
        ctx = self.data_ctx 
        robotid = str(self.robot.id)
        self.stimuli_writer = DataWriter("Robot", ctx, now, robotid)
        # -- Init dist writer -- 
        ctx.name = "DistanceToTasks"
        self.dist_writer = DataWriter("Robot", ctx, now, robotid)
        # Init sensitization writer
        ctx.name = "Sensitizations"
        self.sensitization_writer = DataWriter("Robot", ctx, now, robotid)
        # Init sensitization writer
        ctx.name = "Urgency"
        self.urgency_writer = DataWriter("Robot", ctx, now,robotid)
        
        # raw pose  
        ctx.name = "PoseAtTS"
        ctx.label = "TimeStamp;HH:MM:SS;StepCounter;SelectedTask;X;Y;Theta \n"
        self.pose_writer = DataWriter("Robot", ctx, now, robotid)

        # taskinfo   
        ctx.name = "TaskInfo"
        ctx.label = "TimeStamp;HH:MM:SS;StepCounter;SelectedTask;\
         TaskInfoCount;TaskInfo \n"
        self.taskinfo_writer = DataWriter("Robot", ctx, now, robotid)

    
    def GetCommonHeader(self):
        sep = self.data_ctx.sep
        ts = str(time.time()) + sep + time.strftime("%H:%M:%S", time.gmtime())
        header = ts  + sep + str(self.step) + sep +\
         str(self.selected_taskid)
        return header

    def GetTaskLog(self, log_type):
        log = self.GetCommonHeader()
        sep = self.data_ctx.sep
        taskrec = self.robot.taskrec

        if log_type == LOG_TYPE_STIMULUS:
            for i in range(len(taskrec)):
                log += sep
                log += str(taskrec[i].stimuli)
        elif log_type == LOG_TYPE_DIST:
            for i in range(len(taskrec)):
                log += sep                
                log += str(taskrec[i].dist)
        elif log_type == LOG_TYPE_SENSITIZATION:
            for i in range(len(taskrec)):
                log += sep
                log += str(taskrec[i].sensitization)
        elif log_type == LOG_TYPE_URGENCY:                
            for i in range(len(taskrec)):
                log += sep
                log += str(taskrec[i].urgency)
        else:
            logger.warn("GetTaskLog(): Unknown log type")
        log +="\n"
        return log
    
    def AppendTaskInfoLog(self, taskinfo):        
        sep = DATA_SEP
        length = len(taskinfo)        
        task_ids = taskinfo.keys()
        task_ids.sort() 
        log = self.GetCommonHeader()\
         + sep + str(length) + sep + str(task_ids) + "\n"
        try: 
            self.taskinfo_writer.AppendData(log)
        except:
            logger.warn("TaskInfo logging failed")	
            print "TaskInfo logging failed"

    def AppendTaskLogs(self):
        try:
            self.stimuli_writer.AppendData(self.GetTaskLog(LOG_TYPE_STIMULUS))
            self.dist_writer.AppendData(self.GetTaskLog(LOG_TYPE_DIST))
            self.urgency_writer.AppendData(self.GetTaskLog(LOG_TYPE_URGENCY))
            self.sensitization_writer.AppendData(\
                self.GetTaskLog(LOG_TYPE_SENSITIZATION))            
        except Exception, e:
            print "Task logging failed: ", e
            logger.warn("Task logging failed")
            
    def AppendPoseLog(self):        
        sep = self.data_ctx.sep
        p= self.robot.pose
        log = self.GetCommonHeader()\
         + sep + str(p.x) + sep + str(p.y) + sep + str(p.theta) + "\n"
        try: 
            self.pose_writer.AppendData(log)
        except:
            print "Pose logging failed"
            logger.warn("Pose logging failed")
    
    def SelectTask(self):
        self.CalculateProbabilities()
        end = self.ConvertProbbToRange()
        self.GetRandomSelection(end)

# main process function
def  selector_main(dataManager, robot):
    ts = TaskSelector(dataManager,  robot)
    ts.InitLogFiles()
    ts.datamgr.mRobotPoseAvailable.wait()
    ts.datamgr.mTaskInfoAvailable.wait()
    try:
        for i in range(TASK_SELECTION_STEPS):
            ts.step = i
            logger.info("@TS  ----- [Task Selection Step %d Start ] -----",  i)
            #logger.debug("@TS Robot pose %s:" ,dataManager.mRobotPose.items() )
            ts.SelectTask() # can be started delayed
            ts.PostTaskSelection()
            ts.AppendTaskLogs()
            ts.AppendPoseLog()
            dataManager.mTaskTimedOut.wait() # when task done == timedout
            #time.sleep(5)
    except (KeyboardInterrupt, SystemExit):
            print "User requested exit... TaskSelector shutting down now"
            sys.exit(0)
        
