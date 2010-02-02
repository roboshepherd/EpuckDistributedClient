#!/usr/bin/python
import multiprocessing
from multiprocessing import *
import time
import sys
import  logging,  logging.config,  logging.handlers
logging.config.fileConfig("\
/home/newport-ril/centralized-expt/EpuckCentralizedClient/logging.conf")
logger = logging.getLogger("EpcLogger")
multiprocessing.log_to_stderr(logging.DEBUG)

from RILCommonModules import *
from EpuckCentralizedClient import *
from EpuckCentralizedClient.data_manager import *
from EpuckCentralizedClient.ril_robot import *
#import EpuckCentralizedClient.dbus_server
from EpuckCentralizedClient.dbus_listener import *
from EpuckCentralizedClient.dbus_emitter import *
from EpuckCentralizedClient.task_selector import *
from EpuckCentralizedClient.device_controller import *


def main():
	logging.debug("--- Start EPC---")
	#dbus_server.start()
	dbus_listener.start()
	device_controller.start()
	taskselector.start()
	dbus_emitter.start()
	# Ending....
	time.sleep(2)
	#dbus_server.join()
	try:
		dbus_listener.join()
		device_controller.join()
		taskselector.join()
		dbus_emitter.join()
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
        robotid = sys.argv[1]
	# setup processes
	dbus_shared_path = DBUS_PATH_BASE + robotid
	dm = DataManager(int(robotid))
	##----------START TEST CODE ----#
	#dm.mSelectedTask[SELECTED_TASK_ID] = 1
	#dm.mSelectedTask[SELECTED_TASK_STATUS] = TASK_SELECTED
	#dm.mSelectedTask[SELECTED_TASK_INFO] = [1200000, 1507, 944, 0.0, 0.5]
	#dm.mTaskInfo[1] = [1200000, 1507, 944, 0.0, 0.5]
	## -- END TEST CODE --------#
	
	robot = RILRobot(int(robotid))
	robot.InitTaskRecords(MAX_SHOPTASK)
	sig1 = SIG_ROBOT_POSE 
	sig2 = SIG_TASK_INFO
	sig3 = SIG_TASK_STATUS
	delay = 3 # interval between signals
	#dbus_server = multiprocessing.Process(\
		#target=dbus_server.server_main,
		#name="SwisTrackProxy",\
		#args=(DBUS_IFACE_TRACKER, dbus_shared_path,  sig1,  delay,))
	dbus_listener = multiprocessing.Process(\
		target=listener_main,\
		name="DBusListener",\
		args=(dm,  DBUS_IFACE_TRACKER, dbus_shared_path,\
			DBUS_IFACE_TASK_SERVER, DBUS_PATH_TASK_SERVER,\
			sig1,  sig2,  delay,))
	taskselector = multiprocessing.Process(\
		target=selector_main,\
		name="TaskSelector",\
		args=(dm,  robot))
	dbus_emitter = multiprocessing.Process(\
		target=emitter_main,\
		name="DBusEmitter",\
		args=(dm,  DBUS_IFACE_EPUCK, dbus_shared_path,  sig3,   delay,))
	device_controller =  multiprocessing.Process(\
		target=controller_main,\
		name="DeviceController",  
		args=(dm,))
	main()
      


