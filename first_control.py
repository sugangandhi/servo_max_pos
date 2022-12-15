#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    [py demo]  muliti robot sync test
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
import socket
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
#----------------client comm.-------------------------------------------------
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 8080))
s.listen(5)

#----------------thread 2:check IO---------------------------------------------
def fn_th_check_io():
    if get_digital_input(11) == 0:
        change_operation_speed(10)
        exit()
        wait(0.1)
        return 0

#------------------------------------------------------------------------------
def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        index = get_last_alarm().index
        print(str(index)) 
        level = get_last_alarm().level
        print(str(level))
        group = get_last_alarm().group
        print(str(group))
        param = get_last_alarm().param
        print(str(param))
        #print("  robot_state       : %d" % (msg.robot_state))
        #print("  robot_state_str   : %s" % (msg.robot_state_str))
        #print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))       
        #print("  io_control_box    : %d" % (msg.io_control_box))
        #print("  io_modbus         : %d" % (msg.io_modbus))
        #print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ robot_id + robot_model +'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)      

#--------------------------main--------------------------------------------
if __name__ == "__main__":
    rospy.init_node('first_control_py')
    rospy.on_shutdown(shutdown)

    robot_id = "dsr01"; robot_model = "a0509"

    pub_stop = rospy.Publisher('/'+ robot_id + robot_model +'/stop', RobotStop, queue_size=10)                     

    t = threading.Thread(target=thread_subscriber)
    t.daemon = True 
    t.start()

#----------------------------------------------------------------------
    while not rospy.is_shutdown():
        p2 = posj(-25, 0, 10, 0, 0, 0)  #given position in relative mode
        print("Given position: ",p2)

        pos1 = list(get_current_posj())
        print("Current position: ",list(pos1))

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
        print("max limit from home:  ",list(max))

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
        print("pos to cover max limit from home: ",n_list)

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
        print("max limit from current pos:  ",max1)

        a = 0
        b = 1
        while True:
            if a != b:
                a = get_current_posj()
                movej(p2, vel=60, acc=30, mod = DR_MV_MOD_REL)
                b = get_current_posj() 
                print(b)   
            else:
                pos1 = list(get_current_posj())
                print("Current position: ",list(pos1))
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
                param = get_last_alarm().param
                clientsocket, address = s.accept()
                clientsocket.send(bytes(str(param), "utf-8"))
#----------------------------------------------------------------------
    print('good bye!')
