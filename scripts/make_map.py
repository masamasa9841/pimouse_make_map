#
#[make_map.py]
#
#Copyright (c) [2017] [Masaya Okawa]
#
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php
#
#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import itertools
import signal
import time
import rospy
import numpy as np
from raspimouse_ros.srv import *
from raspimouse_ros.msg import *
from std_msgs.msg import UInt16
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

class left_hand(object):
    def __init__(self):
        rospy.init_node('left_hand')
        signal.signal(signal.SIGINT, self.handler)
        subls = rospy.Subscriber('/lightsensors', LightSensorValues, self.lightsensor_callback, queue_size = 1)
        subsw = rospy.Subscriber('/switches', Switches, self.switch_callback)
        self.sensor = [0, 0, 0, 0]
        self.switch = [0, 0, 0]
        self.__init__rviz()
        time.sleep(1)
        rospy.loginfo("start")

    def __init__rviz(self):#rviz__init__
        self.pub_map = rospy.Publisher('/pimouse_make_map', OccupancyGrid, queue_size=10)
        self.header = Header()
        self.pose = Pose()
        self.point = Point()
        self.quaternion = Quaternion()
        self.info = MapMetaData()
        self.bar = OccupancyGrid()
        self.header.seq = 0
        self.header.frame_id = 'map'
        self.point.x = 0.0
        self.point.y = 0.0 
        self.point.z = 0.0
        self.quaternion.x = 0.0
        self.quaternion.y = 0.0
        self.quaternion.z = 0.0
        self.quaternion.w = 1.0
        self.pose.position = self.point
        self.pose.orientation = self.quaternion
        self.info.width = 76
        self.info.height = 76
        self.info.resolution = 0.1
        self.info.origin = self.pose
        self.map_data = self.read_map_type_data()
        self.array = self.read_map_origin_data()

    def handler(self, signal, frame):
        if not self.switch_motors(False): 
            print "[check failed]: motors are not turned off"
        sys.exit(0)

    def switch_motors(self, onoff):
        rospy.wait_for_service('/switch_motors')
        try:
            p = rospy.ServiceProxy('/switch_motors', SwitchMotors)
            res = p(onoff)
            return res.accepted
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        else:
            return False
    
    def lightsensor_callback(self, msg):
        self.left_side = msg.left_side
        self.right_side = msg.right_side
        print msg.left_side, msg.right_side
        if msg.right_forward > 700: self.sensor[2] = True
        else : self.sensor[2] = False
        if msg.right_side > 400: self.sensor[3] = True
        else : self.sensor[3] = False
        if msg.left_forward > 400: self.sensor[1] = True
        else : self.sensor[1] = False
        if msg.left_side > 700: self.sensor[0] = True
        else : self.sensor[0] = False

    def switch_callback(self, msg): 
        self.switch[0] = msg.front
        self.switch[1] = msg.center
        self.switch[2] = msg.rear
        #print self.switch

    def oneframe(self, left, right, p, dis):
        t = ( 420 * dis ) / ( 2 * math.pi * 2.4 * p) 
        now = rospy.get_time()
        after = 0
        E = 0
        while not after - now >= t and not rospy.is_shutdown():
            after = rospy.get_time()
            if left and right:
                E = 0.15 * (self.left_side - self.right_side)
            elif left and not right:
                E = 0.3 * (self.left_side - 840)
            elif not left and right:
                E = -0.3 * (self.right_side - 840)
            else: E = 0
            if after - now >= t * 2 / 3:
                E = 0
            self.raw_control(p+E,p-E)
            rospy.Rate(100)
            left , right = self.sensor[0], self.sensor[3]

    def turn(self,p, deg, rorl, rl=0):
        self.raw_control(0,0)
        rospy.sleep(0.1)
        t = (deg * 400 * 4.8) / (360 * 2.4 * p)
        if(0 > rorl):
            self.raw_control(p,-p)
            rospy.sleep(t)
        else:
            self.raw_control(-p,p)
            rospy.sleep(t)
        self.raw_control(0,0)
        rospy.sleep(0.1)

    def raw_control(self, left_hz, right_hz):
        pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        if not rospy.is_shutdown():
            d = MotorFreqs()
            d.left = left_hz
            d.right = right_hz
            pub.publish(d)

    def recognition(self, a=False, b=False, type = 0):
        if a and self.sensor[1] and self.sensor[2] and not b:           type = 1
        elif not a and self.sensor[1] and self.sensor[2] and b:         type = 2
        elif a and not self.sensor[1] and not self.sensor[2] and b:     type = 3
        elif a and self.sensor[1] and self.sensor[2] and b:             type = 4
        elif a and not self.sensor[1] and not self.sensor[2] and not b: type = 6#
        elif not a and not self.sensor[1] and not self.sensor[2] and b: type = 5#
        elif not a and self.sensor[1] and self.sensor[2] and not b:     type = 7
        else : print "nothing"
        return type

    def read_map_origin_data(self, count = 0):
        f = open('../map/map_origin.pgm','r')
        map_origin = f.readlines()
        length = map_origin[2].split(" ")
        for i in range(4,len(map_origin)):
            map_origin[i] = map_origin[i].replace('\n','')
            map_origin[i] = int(map_origin[i])
        del map_origin[0:4]
        xy= np.array([[0 for i in range(int(length[0]))]for j in range(int(length[1]))])
        for y in range(int(length[1])):
            for x in range(int(length[0])):
                xy[y][x] = map_origin[count]
                count += 1
        f.close()
        xy = np.where(xy == 0,-1,100)
        return xy

    def read_map_type_data(self):
        ty = [[],[],[],[],[],[],[],[],[]]
        for i in range(9):
            f = open('../map/'+str(i)+'.pgm','r')
            hoge = f.readlines()
            for j in range(4,len(hoge)):
                hoge[j] =  hoge[j].replace('\n','')
                hoge[j] = int(hoge[j])
            del hoge[0:4]
            ty[i] = hoge
            f.close()
        return ty

    def inversion_matrix(self, type = 3, deg = 270, count = 0):
        a = self.map_data[type]
        xy= np.array([[0 for i in range(19)]for j in range(19)])
        if deg == 0:
            for x in range(19):
                for y in range(18,-1,-1):
                    xy[y][x] = a[count]
                    count += 1
        elif deg == 90:
            for y in range(19):
                for x in range(19):
                    xy[y][x] = a[count]
                    count += 1
        elif deg == 190:
            for x in range(18,-1,-1):
                for y in range(19):
                    xy[y][x] = a[count]
                    count += 1
        elif deg == 270:
            for y in range(18,-1,-1):
                for x in range(18,-1,-1):
                    xy[y][x] = a[count]
                    count += 1
        return xy

    def rviz(self, type = 0, i = 0,j = 0, deg = 90):
        self.header.seq += 1
        self.header.stamp = rospy.Time.now()
        self.info.map_load_time = rospy.Time.now()
        self.bar.info = self.info
        self.bar.header = self.header
        self.array[i:i+19,j:j+19] = self.inversion_matrix(type, deg) #deg
        self.bar.data = list(itertools.chain(*self.array))

    def test(self, deg=90, x=0, y=0):
        if not self.switch_motors(True):
            print "[check failed]: motors are not empowered"
        self.rviz(8,y,x,deg)
        self.pub_map.publish(self.bar)
        while not rospy.is_shutdown():
            self.pub_map.publish(self.bar)
            if deg == 360: deg = 0 
            if deg == 450: deg = 90
            if deg <  0: deg = 270
            if deg == 0: x+= 19
            if deg == 90: y+= 19
            if deg == 180: x-= 19
            if deg == 270: y-= 19
            left, right = self.sensor[0], self.sensor[3]
            self.oneframe(left, right, 600, 18)
            #self.raw_control(0,0)
            #rospy.sleep(0.2)
            self.rviz(self.recognition(left, right),y,x,deg)
            front = self.sensor[1]
            if not left:
                self.turn(650, 90, 1)
                deg += 90
            elif left and not front:
                pass
            elif left and right and front:
                self.turn(650, 180, -1)
                deg += 180
            elif left and front:
                self.turn(650, 90, -1)
                deg -= 90

if __name__ == '__main__':
    lh = left_hand()
    try:
        lh.test()
    except:
        if not lh.switch_motors(False): 
            print "[check failed]: motors are not turned off"
