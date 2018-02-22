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

        #
        # receive everyones' joint data 
        #
        values = message.getValues()
        jointname = message.address[1:-len('_joint_global')]
        joint_index = _BrekelJointnameToIndex[jointname]
        body_index = values[0]
        global Body
        global X
        global Y
        global Z
        
        #
        # Condition one:
        #    only receive the joint head's data, joint_index(4) --> head
        #
        if joint_index == 4:
            # define x,y,z 
            x = values[3]
            y = values[4]
            z = values[5]
            #
            # condition two:
            #   only receive the data, which distance is smaller than 3 meters and
            #   the elevation degree is smaller than 30 
            #
            # important arguments for elevation:
            #   0.32 --> horizontal distance between camera and robothead
            #   0.5  --> vertical distance between camera and robothead
            #   1.6  --> vertical distance between camera and ground
            #   1.1  --> vertical distance between robothead and ground 
            elevation = self.elevation(0.32,0.5,1.6,1.1,x,y,z)
            if( (x**2+z**2)<9 and abs(horizontalAngle)<30 ):

                # push_back the wanted body_index, x, y, z into the lists respectively
                Body.append(values[0])
                X.append(values[3])
                Y.append(values[4])
                Z.append(values[5])
            
                # because the camera is 30 fps, when we have received 6 data, it costs 0.2s
                # detect max 6 people
                if len(Body) == 6:
                            
                    # find the nearst people
                    Dist = np.linalg.norm(zip(X,Z),axis=1)
                    Index = Dist.argmin();
                    body = Body[Index];x = X[Index];y = Y[Index];z = Z[Index];
                    # compute degree
                    horizontalAngle = self.horizontalAngle(0.32,0.5,x,z)
                    elevation = self.elevation(0.32,0.5,1.6,1.1,x,y,z)

                    if(len(set(Body)) == 2):
                            # two different people in the scene
                            x1 = X[-1]; y1 = Y[-1]; z1 = Z[-1];
                            x2 = X[-2]; y2 = Y[-2]; z2 = Z[-2];
                            diff = (x1-x2)**2 + (z1-z2)**2
                            if(diff <= 0.8**2):
                                # it means two people are close enough, so robot needs to watch the two at the 
                                # same time
                                horizontalAngle = self.horizontalAngle(0.32,0.5,x1,z1)
                                elevation = self.elevation(0.32,0.5,1.6,1.1,x1,y1,z1)
                                print("I am watching the first ID:%s, horizon:%s, elevation:%s" %(Body[-1], horizontalAngle,elevation))
                                hello_str = str(horizontalAngle)+ ";" + str(elevation)
                                pub.publish(hello_str)
                                rospy.sleep(0.8);
                                horizontalAngle = self.horizontalAngle(0.32,0.5,x2,z2)
                                elevation = self.elevation(0.32,0.5,1.6,1.1,x2,y2,z2)
                                print("I am watching the second ID:%s, horizon:%s, elevation:%s" %(Body[-2], horizontalAngle,elevation))
                                hello_str = str(horizontalAngle)+ ";" + str(elevation)
                                pub.publish(hello_str)
                                rospy.sleep(0.8); 
                                # free the lists                  
                                del Body[:]; del X[:]; del Y[:]; del Z[:]

                            else:
                                # it means two people are not close enough
                                print("the nearst people ID:%s, horizon:%s, elevation:%s" %(body, horizontalAngle,elevation))
                                hello_str = str(horizontalAngle)+ ";" + str(elevation)
                                pub.publish(hello_str)
                                # free the lists
                                del Body[:]; del X[:]; del Y[:]; del Z[:]

                    else:
                        # there are people in the camera expect only two people
                        print("the nearst people ID:%s, horizon:%s, elevation:%s" %(body, horizontalAngle,elevation))
                        hello_str = str(horizontalAngle)+ ";" + str(elevation)
                        pub.publish(hello_str)
                        # free the lists
                        del Body[:]; del X[:]; del Y[:]; del Z[:]
            else:
                # no suitable people in the camera, so send reset signal
                hello_str = "0;0"
                pub.publish(hello_str)
                
        
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
        degree = 2*90*rad/math.pi
        return degree
        
if __name__ == '__main__':
   
	pub = rospy.Publisher('head_position', String, queue_size=10)
	rospy.init_node('head_node', anonymous=True)
	UDPReceiverApplication(17780)
	reactor.run()


