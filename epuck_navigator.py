import  time
import sys
from math import fabs, atan2, atan

from myro import Epuck
from RILCommonModules.RILSetup import *
from RILCommonModules.pose import *
from RILCommonModules.LiveGraph import *
from EpuckDistributedClient.utils import *
import  logging,  logging.config,  logging.handlers
#logging.config.fileConfig("logging.conf")
logger = logging.getLogger("EpcLogger")

PROXIMITY_THRESHOLD = 200
TRANSLATE_SPEED = 0.5
ROTATE_SPEED = 0.644 # 1.0 == 3.14 rev/s, 0.644 == 2.04 rev/s
STEP_DIST = 40 # pixel
TRANSLATE_CONST = 40 # dividing desired px-dist by this gives time(translate) 
ROTATE_CONST = 0.637/2 # mutiplying this with desired angle gives time(rotate)
# typical value: 0.637  max val = 0.411
FORWARD_STEP_TIME = 1
# Obstacle Avoidance params
BACKWARD_SPEED1 = -0.8 # orig: -0.4
BACKWARD_TURN = 0.5 # orig: 0 , no turn
TINY_SLEEP = 0.05 # orig: 0.05

class NavFunc :
    NOTSET = -1
    MINZ = 1,  # minimize axis value
    FIXD = 50 # keep axis value const
    MAXZ = 100  #maximize axis value

class  Quad:
    Q0 = 0 #unknown
    Q1 = 1 #top left
    Q2 = 2 #top right
    Q3 = 3 #bottom right
    Q4 = 4 #bottom left
    Q12 = 12
    Q23 = 23
    Q34 = 34
    Q41 = 41
class EpuckNavigator:
    """Move epuck to a target task boundary using gps like robot pose data """
    def __init__(self):
        self.mRobotPose = Pose()
        self.mTaskPose = Pose()
        self.mTaskRadius = TASK_RADIUS
        self.mTaskConeAngle =  TASK_CONE_ANGLE
        self.mXFunc = NavFunc.NOTSET
        self.mYFunc = NavFunc.NOTSET
        self.mCurrentQuad = Quad.Q0
        self.mThetaDesired = 0.0
        self.mThetaLocal = 0.0
        # epuck
        self.epuck = None
        # navigator log
        self.motion_writer = None
        self.step = 0
        self.translated = 0
        self.rotate_dir = 0
        self.rotate_angle = 0
        self.obstacle_sensor = -1
        # init stuff
        self.InitLogFiles() 


    def InitLogFiles(self):
        # -- Init Stimuli writer --
        name = "Navigation"
        now = time.strftime("%Y%b%d-%H%M%S", time.gmtime())
        desc = "Started from: " + now + "\n"
        desc += "#" + "TRANSLATE_SPEED: " + str(TRANSLATE_SPEED) +"rev rad/s"\
                "\tTRANSLATE_CONST: " + str(TRANSLATE_CONST) + "\n"
        desc += "#" + "TRANSLATE STEP_DIST: " + str(STEP_DIST) + " pixel"+\
                "\tFORWARD_STEP_TIME: " + str(FORWARD_STEP_TIME) + "s \n"
        desc += "#" + "ROTATE_SPEED: " + str(ROTATE_SPEED) +"rev rad/s"\
                "\tROTATE_CONST: " + str(ROTATE_CONST) + "\n"
        desc += "#" + "Valid Coordinates: 1(topLeft)), 2, 3(bottomRight), 4,\
         12, 23, 34, 41\n"
        desc += "#" + "TranslateDone: 1, : NoTranslation: 0 \n" 
        desc += "#" + "RotateDir:  Left== -1,  Right== 1 \n"
        desc += "#" + "Obstacle sensed: -1 : None, (0-7): Sensor#, 8: All"
        # prepare label
        label = "TimeStamp;Step#;Coordinate;Translated;RotateDir;Angle(rad);\
         ObstacleSensor# \n"
        # Data context
        ctx = DataCtx(name, label, desc)
        self.motion_writer = DataWriter("Robot", ctx, now)

    def GetCommonHeader(self):
        ts = time.strftime("%H:%M:%S", time.gmtime())
        sep = DATA_SEP
        self.step = self.step + 1
        header = str(ts) + sep + str(self.step)
        return header
    
    def AppendMotionLog(self):        
        sep = DATA_SEP
        log = self.GetCommonHeader()\
         + sep + str(self.mCurrentQuad) + sep + str(self.translated)\
         + sep + str(self.rotate_dir)\
         + sep + str(self.rotate_angle) + sep + str(self.obstacle_sensor) +"\n"
        try: 
            self.motion_writer.AppendData(log)
        except:
            logger.warn("Motion logging failed")
        # reset to default values
        self.translated = 0
        self.rotate_dir = 0
        self.rotate_angle = 0
        self.obstacle_sensor = -1 
    
    
    def GoForward(self, e, timeout = FORWARD_STEP_TIME):
        self.epuck = e
        maxtime = time.time() + timeout
        threshold = PROXIMITY_THRESHOLD
        # sense obstacle
        sensors = e.getProximity()
        #print sensors
        maxval = max(sensors)
        if maxval > threshold:
            mostActive = sensors.index(maxval)
            allActive = [i for i in range(8) if sensors[i] > PROXIMITY_THRESHOLD]
            logger.info("\t Proximity sensor %d activated", mostActive)
            self.obstacle_sensor = mostActive
            #print 'i feel sensors', allActive
            # flash the closest LED
            led = e.getLEDnumber(mostActive)
            e.flashLED(led, 0.25)
            # response
            if allActive == [0, 7]:
                self.obstacle_sensor = 8 # all
                trans = e.getTranslate()
                rotate = e.getRotate()
                e.move(0, 0.7)
                wait(1)
                e.move(trans, rotate)
            elif mostActive in (0, 7):
                e.move(BACKWARD_SPEED1 , BACKWARD_TURN)
            elif mostActive == 1:
                e.move(-0.3, 0.1)
            elif mostActive == 6:
                e.move(-0.3, -0.1)
            elif mostActive == 2:
                e.move(0.5, 0.2)
            elif mostActive == 5:
                e.move(0.5, -0.2)
            elif mostActive == 3:
                e.move(0.4, 0.2)
            elif mostActive == 4:
                e.move(0.4, -0.2)
            wait(FORWARD_STEP_TIME)
            e.stop() # stop
        else:
            # nothing detected
            e.forward(TRANSLATE_SPEED, FORWARD_STEP_TIME)
        #wait(TINY_SLEEP)
        # log translation
        self.translated = 1
        

    def GoTowardsTarget(self, epuck, rx, ry, rtheta, task_x, task_y, maxtime):
        self.epuck = epuck
        self.UpdateCurrentPose(rx, ry, rtheta)
        self.SetupTaskLoc(task_x,  task_y)
        self.UpdateTargetTaskAngle()
        # update objective function
        self.UpdateNavFunc()
        logger.debug("GoTowardsTarget(): START: %s", time.ctime())
        logger.debug("TaskPoseOrientation:%2f Cone:%2f",\
            self.mTaskPose.theta, self. mTaskConeAngle)
        # Do Rotation by Quad 
        GoActionSwitch = {"12" : self.GoQ12, "23": self.GoQ23,\
            "34": self.GoQ34,  "41":  self.GoQ41,"1": self.GoQ1,\
             "2": self.GoQ2, "3": self.GoQ3, "4": self.GoQ4 }
        GoActionSwitch.get(str(self.mCurrentQuad),  self.GoErrHandler)()
        # Trasnslation is fixed
        time.sleep(TINY_SLEEP)
        self.Translate(epuck)
        time.sleep(TINY_SLEEP) # to stabilize pose
        
    def SetupTaskLoc(self, x, y, r=TASK_RADIUS, ca=TASK_CONE_ANGLE):
        self.mTaskPose.x = x
        self.mTaskPose.y= y
        self.mTaskRadius = r
        self.mTaskConeAngle = ca
        logger.info("Updated TaskLoc: x= %f y= %f", x, y)

    def UpdateCurrentPose(self,  x,  y,  theta):
        self.mRobotPose.x = x
        self.mRobotPose.y = y
        self.mRobotPose.theta= theta
        logger.info( "Updated RobotPose: x= %f y= %f  theta:%f", x, y, theta)

    def UpdateTargetTaskAngle(self):
        dx = fabs(self.mRobotPose.x - self.mTaskPose.x)
        dy = fabs(self.mRobotPose.y - self.mTaskPose.y)
        self.mTaskPose.theta= atan(dy/dx)

    def UpdateNavFunc(self):
        rx = self.mRobotPose.x
        ry = self.mRobotPose.y
        tx1 = self.mTaskPose.x - self.mTaskRadius
        ty1 = self.mTaskPose.y - self.mTaskRadius
        tx2 = self.mTaskPose.x + self.mTaskRadius
        ty2 = self.mTaskPose.y + self.mTaskRadius
        # reset previous values
        self.mXFunc = NavFunc.NOTSET
        self.mYFunc = NavFunc.NOTSET
        self.mCurrentQuad = Quad.Q0
        # for four vertical axes
        if ( (rx < tx1)  and  (ry < ty2 ) and  (ry > ty1) ):  # xleft
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.FIXD
            logger.debug("NavFunc set @Q41")
            self.mCurrentQuad = Quad.Q41
        elif ( (rx > tx2)  and  (ry < ty2 )  and (ry > ty1) ) :      #xright
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.FIXD
            logger.debug("NavFunc set @Q23 ")
            self.mCurrentQuad = Quad.Q23
        elif ( (rx < tx2)  and  (rx > tx1 ) and  (ry < ty1) ):  #yup
            self.mXFunc = NavFunc.FIXD
            self.mYFunc = NavFunc.MAXZ
            logger.debug("NavFunc set @Q12 ")
            self.mCurrentQuad = Quad.Q12
        elif ( (rx < tx2)  and  (rx > tx1 ) and  (ry > ty2) ):   #udown
            self.mXFunc = NavFunc.FIXD
            self.mYFunc = NavFunc.MINZ
            logger.debug("NavFunc set @Q34 ")
            self.mCurrentQuad = Quad.Q34
        elif ( (rx < tx1) and  (ry < ty1) ) :  # @Q1 : topleft
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.MAXZ
            self.mCurrentQuad = Quad.Q1
            logger.debug("NavFunc set @Q1 ")        
        elif ( (rx > tx2)  and (ry < ty1) ) :
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.MAXZ
            self.mCurrentQuad = Quad.Q2
            logger.debug("NavFunc set @Q2 ")
        elif ( (rx > tx2)  and  (ry > ty2) ) :
            self.mXFunc = NavFunc.MINZ
            self.mYFunc = NavFunc.MINZ
            self.mCurrentQuad = Quad.Q3
            logger.debug("NavFunc set @Q3 ")
        elif ( (rx < tx1)  and  (ry > ty2) ) :
            self.mXFunc = NavFunc.MAXZ
            self.mYFunc = NavFunc.MINZ
            self.mCurrentQuad = Quad.Q4
            logger.debug("NavFunc set @Q4 ")
        else:
            logger.warn("UpdateNavFunc():Unknown Quad")
    
    
    def  GoQ12(self):
        self.mThetaDesired = ANGLE90
        if (ANGLE90 < self.mRobotPose.theta < ANGLE270):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            self.RotateLeft(rotate)
        elif (ANGLE270 < self.mRobotPose.theta < ANGLE360):
            rotate = ANGLE360 - self.mRobotPose.theta + self.mThetaDesired
            self.RotateRight(rotate)
        elif (ANGLE90 < self.mRobotPose.theta < ANGLE0):
            rotate = self.mThetaDesired - self.mRobotPose.theta
            self.RotateRight(rotate)
        else:
            logger.warn("@Q12: don't know how much to rotate")
    
    def GoQ23(self):
        self.mThetaDesired = ANGLE180
        if (ANGLE180 < self.mRobotPose.theta < ANGLE360):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            self.RotateLeft(rotate)
        elif (ANGLE0 < self.mRobotPose.theta < ANGLE180):
            rotate = self.mThetaDesired - self.mRobotPose.theta  
            self.RotateRight(rotate)
        else:
            logger.warn("@Q23: don't know how much to rotate")
    
    def GoQ34(self):
        self.mThetaDesired = ANGLE270
        if (ANGLE90 < self.mRobotPose.theta < ANGLE270):
            rotate = self.mThetaDesired - self.mRobotPose.theta 
            self.RotateRight(rotate)
        elif (ANGLE270 < self.mRobotPose.theta < ANGLE360):
            rotate = self.mRobotPose.theta - self.mThetaDesired 
            self.RotateLeft(rotate)
        elif (ANGLE90 < self.mRobotPose.theta < ANGLE0):
            rotate = self.mRobotPose.theta + ANGLE90
            self.RotateLeft(rotate)
        else:
            logger.warn("@Q34: don't know how much to rotate")

    def GoQ41(self):
        self.mThetaDesired = ANGLE360
        if (ANGLE180 < self.mRobotPose.theta < ANGLE360):
            rotate = self.mThetaDesired - self.mRobotPose.theta
            self.RotateRight(rotate)
        elif (ANGLE0 < self.mRobotPose.theta < ANGLE180):
            rotate = self.mRobotPose.theta  
            self.RotateLeft(rotate)
        else:
            logger.warn("@Q41: don't know how much to rotate")
    
    def GoQ1(self):
        self.mThetaDesired = self.mTaskPose.theta
        logger.info("GoQ1(): Theta desired %f",  self.mThetaDesired)
        min_angle =  self.mThetaDesired
        max_angle = ANGLE180 + self.mThetaDesired
        if(min_angle < self.mRobotPose.theta < max_angle):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            self.RotateLeft(rotate)
            logger.debug("\t 1st turn choice %f", rotate)
        elif(max_angle < self.mRobotPose.theta < ANGLE360):
            rotate = (ANGLE360 - self.mRobotPose.theta) + self.mThetaDesired
            self.RotateRight(rotate)
            logger.debug("\t 2nd turn choice %f", rotate)
        else:
            rotate = self.mThetaDesired - self.mRobotPose.theta 
            self.RotateRight(rotate)
            logger.debug("\t 3rd turn choice %f", rotate)

    def GoQ2(self):
        self.mThetaDesired = ANGLE180 -  self.mTaskPose.theta
        logger.info("GoQ2(): Theta desired %f", self.mThetaDesired)
        min_angle = self.mThetaDesired
        max_angle = ANGLE360 - self.mTaskPose.theta
        if(min_angle <= self.mRobotPose.theta <= max_angle):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            self.RotateLeft(rotate)
        elif(max_angle <= self.mRobotPose.theta <= ANGLE360):
            rotate = (ANGLE360 - self.mRobotPose.theta) + self.mThetaDesired
            self.RotateRight(rotate)
        else:
            rotate = self.mThetaDesired - self.mRobotPose.theta 
            self.RotateRight(rotate)
        
    def GoQ3(self):
        self.mThetaDesired = ANGLE180  + self.mTaskPose.theta
        logger.info("GoQ3(): Theta desired %f", self.mThetaDesired)
        min_angle = self.mTaskPose.theta
        max_angle = self.mThetaDesired
        if(min_angle <= self.mRobotPose.theta <= max_angle):
            rotate = self.mThetaDesired - self.mRobotPose.theta
            self.RotateRight(rotate)
        elif(ANGLE0 <= self.mRobotPose.theta <= min_angle):
            rotate = self.mRobotPose.theta + ANGLE360 -  self.mThetaDesired
            self.RotateLeft(rotate)
        else:
            rotate = self.mRobotPose.theta -  self.mThetaDesired
            self.RotateLeft(rotate)

    def GoQ4(self):
        self.mThetaDesired = ANGLE360 -  self.mTaskPose.theta
        logger.info("GoQ4(): Theta desired %f", self.mThetaDesired) 
        min_angle = ANGLE180 - self.mTaskPose.theta
        max_angle = self.mThetaDesired
        logger.info("min_angle %f max_angle %f", min_angle, max_angle)
        if(min_angle <= self.mRobotPose.theta <= max_angle):
            rotate = self.mThetaDesired - self.mRobotPose.theta
            #print "robot pose falls w/i min-max"
            self.RotateRight(rotate)
        elif(max_angle <= self.mRobotPose.theta <= ANGLE360):
            rotate = self.mRobotPose.theta - self.mThetaDesired
            #print "robot pose falls w/i max - 360"
            self.RotateLeft(rotate)
        else:
            rotate = self.mRobotPose.theta + self.mTaskPose.theta
            #print "robot pose falls inside third block"
            self.RotateLeft(rotate)

    def GoErrHandler(self): 
        logger.warn("GoErrHandler(): Unknown Quad, can't go ")
    
    #def TurnReverse(self):
        #logger.debug("\t Turning reverse")
        #tm = ROTATE_CONST * ANGLE180
        #self.TurnLeft(tm)

    def RotateLeft(self, angle):
        logger.debug("\t Rotating left for %f rad", angle)
        # for logging
        self.rotate_dir = -1
        self.rotate_angle = angle
        # real stuff
        tm = ROTATE_CONST * angle
        self.TurnLeft(tm)
    
    def RotateRight(self, angle):
        logger.debug("\t Rotating right for %f rad", angle)
        # for logging
        self.rotate_dir = 1
        self.rotate_angle = angle
        # real stuff
        tm = ROTATE_CONST * angle
        self.TurnRight(tm)
    
    def Translate(self, epuck):
        timeout = STEP_DIST / TRANSLATE_CONST
        logger.debug("Now Doing Translation for %f sec", timeout)
        try:
            #self.epuck.forward(TRANSLATE_SPEED, timeout)
            self.GoForward(epuck, timeout)
            time.sleep(TINY_SLEEP)
        except Exception,e:
            logger.error("Translatation failed for %s", e)
    
    def TurnLeft(self, timeout):
        logger.debug("Now Doing Left Rotation for %f sec", timeout)
        try:
            self.epuck.turnLeft(ROTATE_SPEED, timeout)
        except Exception,e:
            logger.error("TurnLeft failed %s", e)
    
    def TurnRight(self, timeout):
        logger.debug("Now Doing Right Rotation for %f sec", timeout)
        try:
            self.epuck.turnRight(ROTATE_SPEED, timeout)
        except Exception,e:
            logger.error("TurnRight failed: %s", e)

