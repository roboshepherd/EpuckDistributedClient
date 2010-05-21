#!/usr/bin/python
from multiprocessing.managers import BaseManager
from multiprocessing import *
import time
import sys
import  logging,  logging.config,  logging.handlers
logging.config.fileConfig("\
/home/newport-ril/centralized-expt/EpuckDistributedClient/logging-remote.conf")
logger = logging.getLogger("EpcLogger")
from EpuckDistributedClient.data_manager import *
from EpuckDistributedClient.device_controller_remote import *
from RILCommonModules.RILSetup import *

class RemoteManager(BaseManager):
  pass

RemoteManager.register('get_target')

def main():
    logging.debug("--- Start EPC---")
    device_controller.start()
    time.sleep(2)
    try:
        device_controller.join()
    except (KeyboardInterrupt, SystemExit):
        logging.debug("--- End EPC---")
        print "User requested exit..ClientMain shutting down now"                
        sys.exit(0)

if __name__ == '__main__':
    # parse robot id
    numargs = len(sys.argv) - 1
    if numargs > 1 or numargs < 1:
        print "usage:" + sys.argv[0] + " <robot id >"
        sys.exit(1) 
    else:
        robotid = int(sys.argv[1])
        DATA_MGR_PORT = EXPT_SERVER_PORT_BASE + robotid
    # connect to server's  data manager
    mgr = RemoteManager(address=(EXPT_SERVER_IP, DATA_MGR_PORT), authkey="123")
    mgr.connect()
    datamgr = mgr.get_target()
    myid = datamgr.GetRobotID()
    if  int(myid) != robotid:
        print "robot id: " + str(robotid) + "and DataMgr port: " +\
        str(DATA_MGR_PORT) + "mismatch -- check both are started..."
        sys.exit(1)
    # setup processes
    device_controller =  Process(\
		target=controller_main,\
		name="DeviceController",  
		args=(datamgr,))
    #print tgt.GetRobotPose()
    #print tgt.GetTaskInfo()
    ##print tgt.IsRobotPoseAvailable()
    ##tgt.SetSelectedTaskStarted()
    #print tgt.IsSelectedTaskAvailable()
    #print tgt.GetSelectedTask()
    main()
    
