#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 30 20:46:52 2018

@author: wanting

    
    'distance_h' is the horizontal distance between kinect and robot head 
    if 'robot head' locates on the right of kinect,'distance_h' is positiv
    
    'distance_v' is the vertical distance between kinect and robot head 
    if 'robot head' locates in the front of kinect,'distance_h' is positiv
    
    """
import math

def horizontalAngle(distance_h,distance_v,x,z):
    rad = -math.atan((distance_h-x)/(z-distance_v))
    degree = 2*90*rad/math.pi
    return degree
    
def elevation(distance_h,distance_v,height_kinect,height_robot,x,y,z):
    distance = math.sqrt((z-distancev)**2+(distanceh-x)**2)
    rad = math.atan(((height_kinect+y)-height_robot)/distance)

#        rad = math.atan(((height_kinect+y)-height_robot)/math.sqrt(z**2+(distance-x)**2))
#        rad = math.atan(y/(math.sqrt(z**2+(distance-x)**2)))
    degree = 2*90*rad/math.pi
    return degree
    

a = horizontalAngle(1,0,0.3,1.2)
