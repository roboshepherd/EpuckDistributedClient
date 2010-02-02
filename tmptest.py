#!/usr/bin/env python

from data_manager import *
from dbus_listener import *
from dbus_emitter import *
dm = DataManager(33)
dm.mLocalTaskInfo = {1: [1200000, 1507, 944, 0.0, 0.5]}
dm.mRobotPeers[ROBOT_PEERS] = [3, 5]
#listener_main(dm)

emitter_main(dm)





