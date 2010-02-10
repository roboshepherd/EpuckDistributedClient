import  logging,  logging.config,  logging.handlers
#logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")

from RILCommonModules.RILSetup import *
from RILCommonModules.pose import *
from RILCommonModules.shop_task import *

from EpuckDistributedClient.data_manager import *

class TaskRecord:
    info = []
    def __init__(self,  id=-1,  sensitization=INIT_SENSITIZATION,\
                   dist=0,  stimuli=0,  probability=0,  urgency=0, timesDone=0):
        self.id = id
        self.sensitization = sensitization
        self.dist = dist
        self.stimuli = stimuli
        self.probability = probability
        self.urgency = urgency
        self.timesDone = timesDone
    def Info(self):
        self.info = [self.id,  self.sensitization,   self.dist,  self.stimuli,  self.probability  ]
        return self.info

class RILRobot:
    def __init__(self,  id=-1,  state=NOTSET,  pose=Pose(),\
                   lr=INIT_LEARN_RATE,  fr=INIT_FORGET_RATE, shoptask=ShopTask() ):             
        self.id = id
        self.state = state # device state
        self.pose = pose
        self.learnrate = lr
        self.forgetrate = fr
        self.shoptask = shoptask # currently doing task
        self.taskrec = {}

    def InitTaskRecords(self,  taskcount):
        """ Initialize task records where task0 is random walk"""
        count = taskcount
        while count >= 0:
             self.taskrec[count] = TaskRecord(id=count)
             count = count - 1
    
    def UpdatePose(self,  datamgr):
        #datamgr.mRobotPoseAvailable.wait()
        #if(datamgr.mRobotPose[0] is not 0): # default value changed
        try:
            self.pose.Update(datamgr.mRobotPose)
        except:
            print "@Robot Raw pose from datamgr: %d" %(len(datamgr.mRobotPose))
        #print self.pose.info
    
    def UpdateTaskRecords(self,  selectedTaskId):
            sz = self.taskrec[selectedTaskId].sensitization
            td = self.taskrec[selectedTaskId].timesDone
           # logger.info("task %d sensitization:%f, timesDone:%d",\
           #                 selectedTaskId,  sz,  td  )
            try:
                for k,  v in self.taskrec.iteritems():   
                    if k is selectedTaskId:
                        self.taskrec[selectedTaskId].sensitization = sz + self.learnrate
                        self.taskrec[selectedTaskId].timesDone  = td + 1
                        logger.info("\ttask %d inc. sensitization:%f, timesDone:%d",\
                                     selectedTaskId,  self.taskrec[selectedTaskId].sensitization,\
                                     self.taskrec[selectedTaskId].timesDone  )
                    else:
                         self.taskrec[k].sensitization = self.taskrec[k].sensitization - self.forgetrate
                    # keep value between 0 ~ 1
                    if self.taskrec[k].sensitization < 0:
                        self.taskrec[k].sensitization = 0
                    elif self.taskrec[k].sensitization > 1:
                        self.taskrec[k].sensitization = 1
            except:
                logger.warn("Task record update failed")
