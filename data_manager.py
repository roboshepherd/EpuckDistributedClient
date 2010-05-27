import time
from multiprocessing import *    
from multiprocessing.managers import BaseManager
from RILCommonModules.RILSetup import *
class DataManager:
    def __init__(self,  id=-1):
        self.mRobotID = id
        self.mgr = Manager()
        self.mRobotPose = self.mgr.dict() # to retrieve last  observed pose
       # key: x, y, theta, time_stamp(ts) val: <values>
        self.mRobotPoseAvailable = self.mgr.Event() # set by dbus client
        self.mTaskInfo = self.mgr.dict() 
        # key: taskid, value: list of attrib (t.s., x, y,  phi)        
        self.mTaskInfoAvailable = self.mgr.Event() # set by dbus client
        self.mSelectedTask = self.mgr.dict()  
        # Set/Unset by TaskSelector, DeviceController
        # key: SEE RILsetup, val: Values        
        self.mSelectedTaskAvailable = self.mgr.Event() 
        # Set/Unset by TaskSelector
        self.mSelectedTaskStarted = self.mgr.Event()
        #set by Device controller, used/clear by dbus_emmitter
        
        # DeviceController Signals
        self.mTaskTimedOut = self.mgr.Event()  # Set/Unset by DeviceController

        # RobotPeers stored as dict:'ts'' = timestamp and 'peers' = []
        self.mRobotPeers = self.mgr.dict()
        self.mRobotPeersAvailable = self.mgr.Event() 

        # local taskinfo: dict key: taskid, val: taskinfo with type-of-info
        # primary, secondary
        self.mLocalTaskInfo = self.mgr.dict()
        
    # accessors
    def GetRobotID(self):
        return self.mRobotID        
    def GetRobotPose(self):
        val = self.mRobotPose.copy()
        return str(val)
    def IsRobotPoseAvailable(self):
        val = False
        if self.mRobotPoseAvailable.is_set():
            val = True
        return val
    def GetTaskInfo(self):
        val = self.mTaskInfo.copy()
        return str(val)
    def IsTaskInfoAvailable(self):
        val = False
        if self.mTaskInfoAvailable.is_set():
            val = True
        return val
    def GetSelectedTask(self):
        val = self.mSelectedTask.copy()
        return str(val) 
    def IsSelectedTaskAvailable(self):
        val = False
        if self.mSelectedTaskAvailable.is_set():
            val = True
        return val
    def IsSelectedTaskStarted(self):
        val = False
        if self.mSelectedTaskStarted.is_set():
            val = True
        return val
    def IsTaskTimedOut(self):
        val = False
        if self.mTaskTimedOut.is_set():
            val = True
        return val

    def GetRobotPeers(self):
        val = []
        if self.IsRobotPeersAvailable():
            val = self.mRobotPeers[ROBOT_PEERS]
        return str(val)    	
    def IsRobotPeersAvailable(self):
        val = False
        if self.mRobotPeersAvailable.is_set():
            val = True
        return val
    def GetLocalTaskInfo(self):
        val = {}
        if self.IsTaskInfoAvailable():
            val = self.mLocalTaskInfo.copy()
            tmp = self.mTaskInfo.copy()
            val.update(tmp)
        return str(val) 
	
## mutators
    def SetRobotID(self, i):
        self.mRobotID = i

    # robot pose 
    def SetRobotPose(self, pose):
        for k, v in pose.iteritems():
            self.mRobotPose[k] = v
    def SetRobotPoseAvailable(self):
        self.mRobotPoseAvailable.set()
    def ClearRobotPoseAvailable(self):
        self.mRobotPoseAvailable.clear()

    # robot peers
    def SetRobotPeers(self, peers):
        for k, v in peers.iteritems():
            self.mRobotPeers[k] = v
    def SetRobotPeersAvailable(self):
        self.mRobotPeersAvailable.set()
    def ClearRobotPeersAvailable(self):
        self.mRobotPeersAvailable.clear()
    
    # task info
    def SetTaskInfo(self, ti):
        for k, v in ti.iteritems():
            self.mTaskInfo[k] = v
    def SetTaskInfoAvailable(self):
        self.mTaskInfoAvailable.set()
    def ClearTaskInfoAvailable(self):
        self.mTaskInfoAvailable.clear()

    # local taskinfo	
    def SetLocalTaskInfo(self, ti):
        for k, v in ti.iteritems():
            self.mLocalTaskInfo[k] = v
    def SetLocalTaskInfoAvailable(self):
        self.mLocalTaskInfoAvailable.set()
    def ClearLocalTaskInfoAvailable(self):
        self.mLocalTaskInfoAvailable.clear()

    # selected task
    def SetSelectedTask(self, task):
        for k, v in task.iteritems():
            self.mSelectedTask[k] = v
    def SetSelectedTaskAvailable(self):
        self.mSelectedTaskAvailable.set()
    def WaitSelectedTaskAvailable(self):
        self.mSelectedTaskAvailable.wait()
    def ClearSelectedTaskAvailable(self):
        self.mSelectedTaskAvailable.clear()
    
    def SetSelectedTaskStarted(self):
        self.mSelectedTaskStarted.set()
    def WaitSelectedTaskStarted(self):
        self.mSelectedTaskStarted.wait()
    def ClearSelectedTaskStarted(self):
        self.mSelectedTaskStarted.clear()
    
    def SetTaskTimedOut(self):
        self.mTaskTimedOut.set()
    def WaitTaskTimedOut(self):
        self.mTaskTimedOut.wait()
    def ClearTaskTimedOut(self):
        self.mTaskTimedOut.clear()

class RemoteManager(BaseManager):
  pass
  
def datamgr_main(dm):
    tgt = dm
    port = EXPT_SERVER_PORT_BASE  + int(dm.mRobotID)
    RemoteManager.register('get_target', callable=lambda:tgt)
    mgr = RemoteManager(address=(EXPT_SERVER_IP, port), authkey="123")
    srv = mgr.get_server()
    srv.serve_forever()

## Note: When getting value from data_manager, we need to use eval()
