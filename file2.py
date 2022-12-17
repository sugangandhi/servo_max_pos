#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
from file1 import user_input
from file3 import kinematics

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

#for single robot 
import DR_init
DR_init.__dsr__id = "dsr01"
DR_init.__dsr__model = "a0509"
from DSR_ROBOT import *

def shutdown():
    print("shutdown time!")
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def file1_output():
    input = user_input()
    return input

def file3_output():
    input = kinematics()
    return input

if __name__ == "__main__":
    rospy.init_node('file2.py')
    rospy.on_shutdown(shutdown)
    robot_id = "dsr01"; robot_model = "a0509"
    pub_stop = rospy.Publisher('/'+ robot_id + robot_model +'/stop', RobotStop, queue_size=10)                     
 
    while not rospy.is_shutdown():
        p2 = file3_output()  #get joint values from file3

        pos1 = list(get_current_posj())

        max = posj(360, 95, 135, 360, 135, 360) #check if given position is negative then change max limit to negative
        if (p2[0]  < 0):
            max[0] = -(max[0])
        if (p2[1] < 0):
            max[1] = -(max[1])
        if (p2[2] < 0):
            max[2] = -(max[2])
        if (p2[3] < 0):
            max[3] = -(max[3])
        if (p2[4] < 0):
            max[4] = -(max[4])
        if (p2[5] < 0):
            max[5] = -(max[5])
        #print("max limit from home:  ",list(max))

        list1 = list(p2)
        list2 = list(max)
        n_list = []
        if list1[0] == 0:
            item = (list1[0] % list2[0])
            n_list.append(item)
        else:
            item = (list2[0] % list1[0])   #calculate remaining position for max limit
            n_list.append(item)
        if list1[1] == 0:
            item = (list1[1] % list2[1])
            n_list.append(item)
        else:
            item = (list2[1] % list1[1])
            n_list.append(item)
        if list1[2] == 0:
            item = (list1[2] % list2[2])
            n_list.append(item)
        else:
            item = (list2[2] % list1[2])
            n_list.append(item)
        if list1[3] == 0:
            item = (list1[3] % list2[3])
            n_list.append(item)
        else:
            item = (list2[3] % list1[3])
            n_list.append(item)
        if list1[4] == 0:
            item = (list1[4] % list2[4])
            n_list.append(item)
        else:
            item = (list2[4] % list1[4])
            n_list.append(item)
        if list1[5] == 0:
            item = (list1[5] % list2[5])
            n_list.append(item)
        else:
            item = (list2[5] % list1[5])
            n_list.append(item)
        #print("pos to cover max limit from home: ",n_list)

        max1 = posj(360, 95, 135, 360, 135, 360)
        if (pos1[0]  < 0):                #check for negative values for current position and changes maximum limit to negative side
            max1[0] = -(max1[0])
        if (pos1[1] < 0):
            max1[1] = -(max1[1])
        if (pos1[2] < 0):
            max1[2] = -(max1[2])
        if (pos1[3] < 0):
            max1[3] = -(max1[3])
        if (pos1[4] < 0):
            max1[4] = -(max1[4])
        if (pos1[5] < 0):
            max1[5] = -(max1[5])
        #print("max limit from current pos:  ",max1)

        a = 0
        b = 1
        while True:
            if a != b:
                a = get_current_posj()
                movej(p2, vel=60, acc=30, mod = DR_MV_MOD_REL)
                b = get_current_posj()   
            else:
                param = get_last_alarm().param
                clientsocket, address = s.accept()
                clientsocket.send(bytes(str(param), "utf-8"))
                pos1 = list(get_current_posj())
                if p2[0] < 0:
                    if pos1[0] < 0:
                        pass
                    else:
                        pos1[0] = -(pos1[0])
                else:
                    if pos1[0] > 0:
                        pass
                    else:
                        pos1[0] = -(pos1[0])
                if p2[1] < 0:
                    if pos1[1] < 0:
                        pass
                    else:
                        pos1[1] = -(pos1[1])
                else:
                    if pos1[1] > 0:
                        pass
                    else:
                        pos1[1] = -(pos1[1])
                if p2[2] < 0:
                    if pos1[2] < 0:
                        pass
                    else:
                        pos1[2] = -(pos1[2])
                else:
                    if pos1[2] > 0:
                        pass
                    else:
                        pos1[2] = -(pos1[2])
                if p2[3] < 0:
                    if pos1[3] < 0:
                        pass
                    else:
                        pos1[3] = -(pos1[3])
                else:
                    if pos1[3] > 0:
                        pass
                    else:
                        pos1[3] = -(pos1[3])
                if p2[4] < 0:
                    if pos1[4] < 0:
                        pass
                    else:
                        pos1[4] = -(pos1[4])
                else:
                    if pos1[4] > 0:
                        pass
                    else:
                        pos1[4] = -(pos1[4])
                if p2[5] < 0:
                    if pos1[5] < 0:
                        pass
                    else:
                        pos1[5] = -(pos1[0])
                else:
                    if pos1[5] > 0:
                        pass
                    else:
                        pos1[5] = -(pos1[5])

                pos = [] 
                if pos1[0] == 0:
                    item = (pos1[0] % max[0])#calculate max limit from home condition
                    pos.append(item)
                else:                       
                    item = max[0] % pos1[0]
                    pos.append(item)
                if pos1[1] == 0:
                    item = (pos1[1] % max[1])
                    pos.append(item)
                else:
                    item = max[1] % pos1[1]
                    pos.append(item)
                if pos1[2] == 0:
                    item = (pos1[2] % max[2])
                    pos.append(item)
                else:
                    item = max[2] % pos1[2]
                    pos.append(item)
                if pos1[3] == 0:
                    item = (pos1[3] % max[3])
                    pos.append(item)
                else:
                    item = max[3] % pos1[3]
                    pos.append(item)
                if pos1[4] == 0:
                    item = (pos1[4] % max[4])
                    pos.append(item)
                else:
                    item = max[4] % pos1[4]
                    pos.append(item)
                if pos1[5] == 0:
                    item = (pos1[5] % max[5])
                    pos.append(item)
                else:
                    item = max[5] % pos1[5]
                    pos.append(item)
                movej(pos, vel=60, acc=30, mod = DR_MV_MOD_REL)