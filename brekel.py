#!/usr/bin/env python
"""
--------script to receive OSC pckages from Brekel Body Pro 2 via UDP-----------

"""
from twisted.internet import reactor, task
from txosc import dispatch
from txosc import async

import rospy
from std_msgs.msg import String

import time
import math
import numpy as np

Body = []; X = []; Y = []; Z = [];

#horizontalAngle = 0

def foo_handler(message, address):
    """
    Function handler for /foo
    """
    print("foo_handler")
    print("  Got %s from %s" % (message, address))


_BrekelJointnameToIndex = {
'waist'      : 0,
'spine'      : 1,
'chest'      : 2,
'neck'       : 3,
'head'       : 4,
'head_tip'   : 5,
'upperLeg_L' : 6,
'lowerLeg_L' : 7,
'foot_L'     : 8,
'toes_L'     : 9,
'upperLeg_R' :10,
'lowerLeg_R' :11,
'foot_R'     :12,
'toes_R'     :13,
'collar_L'   :14,
'upperArm_L' :15,
'foreArm_L'  :16,
'hand_L'     :17,
'collar_R'   :18,
'upperArm_R' :19,
'foreArm_R'  :20,
'hand_R'     :21,
}

_BrekelSpaceSuffixes = [
'_joint_global',
'_joint_local',
'_joint_2D'
]

MAX_NR_BREKELBODIES = 6

_globalLinkCoordinatesElements = [
'timestamp',
'confidence',
'X',
'Y',
'Z',
'RX',
'RY',
'RZ'
]

_localLinkCoordinatesElements = [
'timestamp',
'confidence',
'X',
'Y',
'Z',
'RX',
'RY',
'RZ'
]

_cameraLinkCoordinatesElements = [
'timestamp',
'confidence',
'X',
'Y',
]


class UDPReceiverApplication(object):
    """
    Example that receives UDP OSC messages.
    """
   # self.horiAnglelist  = []
    horizontalAngle=0
    def __init__(self, port):
        # network settings
        self.port = port
        self.receiver = dispatch.Receiver()
        self._server_port = reactor.listenUDP(self.port, async.DatagramServerProtocol(self.receiver))
        print("Listening on osc.udp://0.0.0.0:%s" % (self.port))
        self.receiver.addCallback("/*_joint_global", self.globalFrameHandler)
        self.receiver.addCallback("/*_joint_local", self.localFrameHandler)
        self.receiver.addCallback("/*_joint_2D", self.cameraFrameHandler)
        self.receiver.addCallback("/lean", self.ignoreHandler)
        self.receiver.addCallback("/handState", self.ignoreHandler)
        self.receiver.addCallback("/face2D", self.ignoreHandler)
        self.receiver.addCallback("/", self.ignoreHandler)
        

        self.watchDogTimeout = 0.5
        self.watchdogDeadline = None
        loop = task.LoopingCall(self._watchDogFunction)
        loop.start(1.0)

        # fallback:
        self.receiver.fallback = self.fallback
        
        


    def _watchDogFunction(self):
        if self.watchdogDeadline is not None:
            if self.watchdogDeadline < time.time():
                print("Receiving no data")
                # when it receives no data, it will send (0,0)
                print("Sending reset signal to motor")
                hello_str = "0;0"
                pub.publish(hello_str)
                self.watchdogDeadline = None
                
                #reactor.stop()

    def resetWatchDog(self):
        if self.watchdogDeadline is None:
            print("Receiving body data now.")
        self.watchdogDeadline = time.time() + self.watchDogTimeout

    def globalFrameHandler(self, message, address):
        """
        Method handler for operational space coordinates of links
        """
        self.resetWatchDog()

        values = message.getValues()
        jointname = message.address[1:-len('_joint_global')]
        joint_index = _BrekelJointnameToIndex[jointname]
        body_index = values[0]
        global Body
        global X
        global Y
        global Z
        #time.sleep(0.5)
        
        if joint_index == 4:
            # detect max six different people
            
            Body.append(values[0])
            X.append(values[3])
            Y.append(values[4])
            Z.append(values[5])
            
            if len(Body) == 6:
                
                Dist = np.linalg.norm(zip(X,Z),axis=1)
                Index = Dist.argmin();
                body = Body[Index];
                x = X[Index]
                y = Y[Index]
                z = Z[Index]
                
                if((x**2+z**2) < 9):
                    horizontalAngle = self.horizontalAngle(0,0.4,x,z)
                    elevation = self.elevation(0,0.4,1.6,1.3,x,y,z)
                    if(abs(horizontalAngle) < 30):
                        # transform Body list into set, in order to calculate bodies
                        bodySet = set(Body)
                        if(len(bodySet) == 2):
                            # two different people in the scene
                            x1 = X[-1]; y1 = Y[-1]; z1 = Z[-1];
                            x2 = X[-2]; y2 = Y[-2]; z2 = Z[-2];
                            diff = (x1-x2)**2 + (z1-z2)**2
                            if(diff <= 0.6**2):
                                # it means two people are close enough, so robot needs to watch the two at the 
                                # same time
                                horizontalAngle = self.horizontalAngle(0,0.4,x1,z1)
                                elevation = self.elevation(0,0.4,1.6,1.3,x1,y1,z1)
                                print("I am watching the first ID:%s, Degree:%s" %(Body[-1], horizontalAngle))
                                hello_str = str(horizontalAngle)+ ";" + str(elevation)
                                pub.publish(hello_str)
                                horizontalAngle = self.horizontalAngle(0,0.4,x2,z2)
                                elevation = self.elevation(0,0.4,1.6,1.3,x2,y2,z2)
                                print("I am watching the second ID:%s, Degree:%s" %(Body[-2], horizontalAngle))
                                hello_str = str(horizontalAngle)+ ";" + str(elevation)
                                pub.publish(hello_str)
                                del Body[:]; del X[:]; del Y[:]; del Z[:]

                            else:
                                print("the nearst people ID:%s, Degree:%s" %(body, horizontalAngle))
                                hello_str = str(horizontalAngle)+ ";" + str(elevation)
                                pub.publish(hello_str)
                                # free the lists
                                del Body[:]; del X[:]; del Y[:]; del Z[:]

                        else:
                            # check the info. output
                            print("the nearst people ID:%s, Degree:%s" %(body, horizontalAngle))
                            hello_str = str(horizontalAngle)+ ";" + str(elevation)
                            pub.publish(hello_str)
                            # free the lists
                            del Body[:]; del X[:]; del Y[:]; del Z[:]
                    else:
                        hello_str = "0;0"
                        pub.publish(hello_str)
                        del Body[:]; del X[:]; del Y[:]; del Z[:]
                else:
                    hello_str = "0;0"
                    pub.publish(hello_str)
                    del Body[:]; del X[:]; del Y[:]; del Z[:]
                

            #horizontalAngle = self.horizontalAngle(1,values[3],values[5])
            #horiAnglelist.append(horizontalAngle)
            
            #elevation = self.elevation(1,1.6,1.6,values[3],values[4],values[5])
            #print("body ID: %s, coordinates: x:%s y:%s z:%s" % (values[0], values[3], values[4], values[5]))
            #print("horizontalAngle:%s" % horizontalAngle)
            #hello_str = str(horizontalAngle)
            #pub.publish(hello_str)
            #print("  elevation:%s" % elevation)
            #print("  list:%s" % len(horiAnglelist))
            
#            if(len(horiAnglelist) > 0):
#                horiAnglelist = [0]
#                reactor.stop()
                
        
        """
        data access here
            values is a list of joints
        """

    def localFrameHandler(self, message, address):
        """
        Method handler for opspace coordinates relative to the parent links
        (i.e. joint space coordinates)
        """
        values = message.getValues()
        jointname = message.address[1:-len('_joint_local')]
        joint_index = _BrekelJointnameToIndex[jointname]
        body_index = values[0]
        """
        data access here
            values is a list of joints
        """

    def cameraFrameHandler(self, message, address):
        """
        Method handler for coordinates in the camera image
        """
        values = message.getValues()
        jointname = message.address[1:-len('_joint_2D')]
        joint_index = _BrekelJointnameToIndex[jointname]
        body_index = values[0]
        """
        data access here
            values is a list of joints
        """

    def fallback(self, message, address):
        """
        Fallback for any unhandled message
        """
        print("Fallback:")
        print("  Got unknown %s from %s" % (message, address))

    def ignoreHandler(self, message, address):
        return
    """
    'distance' is the distance between kinect and robot head 
    'robot head' locates on the right of kinect
    """
    def horizontalAngle(self,distance_h,distance_v,x,z):
        rad = -math.atan((distance_h-x)/(z-distance_v))
        degree = 2*90*rad/math.pi
        return degree
    
    def elevation(self,distance_h,distance_v,height_kinect,height_robot,x,y,z):
        distance = math.sqrt((z-distance_v)**2+(distance_h-x)**2)
        rad = math.atan(((height_kinect+y)-height_robot)/distance)
    
    #        rad = math.atan(((height_kinect+y)-height_robot)/math.sqrt(z**2+(distance-x)**2))
    #        rad = math.atan(y/(math.sqrt(z**2+(distance-x)**2)))
        degree = 2*90*rad/math.pi
        return degree
#    def horizontalAngle(self,distance,x,z):
#        rad = math.atan(z/(distance-x))
#        degree = 2*90*rad/math.pi
#        return degree
#        
#    def elevation(self,distance,height_kinect,height_robot,x,y,z):
#        rad = math.atan(((height_kinect+y)-height_robot)/math.sqrt(z**2+(distance-x)**2))
##        rad = math.atan(y/(math.sqrt(z**2+(distance-x)**2)))
#        degree = 2*90*rad/math.pi
#        return degree
        
if __name__ == '__main__':
   
	pub = rospy.Publisher('head_position', String, queue_size=10)
	rospy.init_node('head_node', anonymous=True)
	UDPReceiverApplication(17780)
	reactor.run()


