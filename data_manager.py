import time
from multiprocessing import *    
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
        #self.mDeviceNotResponding = 
