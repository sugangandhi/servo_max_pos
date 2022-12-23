#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sympy as sp
import numpy as np 
from sympy import *
from sympy.physics.mechanics import *
theta1, theta2, theta3, theta4, theta5, theta6, theta,l1,l2,l3,l4, alpha, a, d = dynamicsymbols('theta1, theta2, theta3, theta4, theta5, theta6, l1, l2, l3, l4, theta, alpha, a, d')
init_printing(use_unicode=True)
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)
__ROS__ = True

import rospy
import os
import threading, time
from std_msgs.msg import String,Int32,Int32MultiArray,Float32,Float64,Float32MultiArray,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
import sys
sys.dont_write_bytecode = True


from dsr_msgs.msg import *
from dsr_msgs.srv import *

from math import *

def rotm2eul(rotm, flip=0):

    r11 = rotm[0][0]
    r12 = rotm[0][1]
    r13 = rotm[0][2]
    r21 = rotm[1][0]
    r22 = rotm[1][1]
    r23 = rotm[1][2]
    r31 = rotm[2][0]
    r32 = rotm[2][1]
    r33 = rotm[2][2]

    # calculate rx, ry, rz
    if abs(r13) < sys.float_info.epsilon and abs(r23) < sys.float_info.epsilon:
        rx = 0
        sp = 0
        cp = 1

        ry = atan2(cp * r13 + sp * r23, r33)
        rz = atan2(-sp * r11 + cp * r21, -sp * r12 + cp * r22)
    else:
        if flip == 0:
            rx = atan2(r23, r13)
            # ry = atan2(sqrt(r13 * r13 + r23 * r23), r33)
            # rz = atan2(r32, -r31)
        else: # -> (flip == 1)
            rx = atan2(-r23, -r13)
            #ry = atan2(-sqrt(r13 * r13 + r23 * r23), r33)
            #rz = atan2(-r32, r31)
        
        sp = sin(rx)
        cp = cos(rx)

        ry = atan2(cp * r13 + sp * r23, r33)
        rz = atan2(-sp * r11 + cp * r21, -sp * r12 + cp * r22)
       

    eulv = [np.degrees(rx), np.degrees(ry), np.degrees(rz)]

    return eulv


rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha)],
                 [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha)],
                 [0, sp.sin(alpha), sp.cos(alpha)]])
trans = sp.Matrix([a*sp.cos(theta), a*sp.sin(theta), d])
last_row = sp.Matrix([[0, 0, 0, 1]])

t = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)
t01 = t.subs({alpha:90,   a:0,      theta:0,     d:155.5})
t12 = t.subs({alpha:0,    a:0,  theta:-90,     d:0.0})
t23 = t.subs({alpha:90,  a:0.409,      theta:0,    d:0})
t34 = t.subs({alpha:90,   a:0,      theta:0,     d:367})
t45 = t.subs({alpha:-90,  a:0,      theta:0,    d:0})
t56 = t.subs({alpha:0,    a:0,      theta:0,     d:124})
t06 = (t01*t12*t23*t34*t45*t56)

tbee = sp.Matrix([[simplify(t06[0,0]), simplify(t06[0,1]), simplify(t06[0,2]), sp.trigsimp(t06[0,3])],
                  [simplify(t06[1,0]), simplify(t06[1,1]), simplify(t06[1,2]), simplify(t06[1,3])],
                  [simplify(t06[2,0]), simplify(t06[2,1]), simplify(t06[2,2]), sp.trigsimp(t06[2,3])]])
#print(tbee)
px = (t06[0,3])
print("px:", px)
py = sp.simplify(tbee[1,3])
print("py:", py)
pz = sp.simplify(tbee[2,3])
print("pz:", pz)
R11 = t06[0,0]
R12 = t06[0,1]
R13 = t06[0,2]
R21 = t06[1,0]
R22 = t06[1,1]
R23 = t06[1,2]
R31 = t06[2,0]
R32 = t06[2,1]
R33 = t06[2,2]

rotm =[[R11,R12,R13], [R21,R22,R23], [R31,R32,R33]]
print(rotm2eul(rotm))

x = sp.trigsimp(atan2(R32,R33))
print("rx:",sp.simplify(x))
y = sp.trigsimp(atan2(-R31, (((R32**2) + (R33**2))**(0.5))))
print("ry:",y)
z = sp.trigsimp(atan2(R21,R11))
print("rz:",z)


# alpha = 0
# a = 0
# theta = 50
# d = 155.5
# tt01 = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.cos(np.deg2rad(theta))],
#                  [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), -np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.sin(np.deg2rad(theta))],
#                  [0, np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), d],
#                  [0, 0, 0, 1]])
# alpha = -90
# a = 0
# theta = -90
# d = 0
# tt12 = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.cos(np.deg2rad(theta))],
#                  [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), -np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.sin(np.deg2rad(theta))],
#                  [0, np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), d],
#                  [0, 0, 0, 1]])
# alpha = 0
# a = 409
# theta = 90
# d = 0
# tt23 = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.cos(np.deg2rad(theta))],
#                  [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), -np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.sin(np.deg2rad(theta))],
#                  [0, np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), d],
#                  [0, 0, 0, 1]])
# alpha = 90
# a = 0
# theta = 10
# d = 367
# tt34 = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.cos(np.deg2rad(theta))],
#                  [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), -np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.sin(np.deg2rad(theta))],
#                  [0, np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), d],
#                  [0, 0, 0, 1]])
# alpha = -90
# a = 0
# theta = 0
# d = 0
# tt45 = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.cos(np.deg2rad(theta))],
#                  [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), -np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.sin(np.deg2rad(theta))],
#                  [0, np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), d],
#                  [0, 0, 0, 1]])
# alpha = 90
# a = 0
# theta = 0
# d = 124
# tt56 = np.array([[np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.cos(np.deg2rad(theta))],
#                  [np.sin(np.deg2rad(theta)),  np.cos(np.deg2rad(theta))*np.cos(np.deg2rad(alpha)), -np.cos(np.deg2rad(theta))*np.sin(np.deg2rad(alpha)), a*np.sin(np.deg2rad(theta))],
#                  [0, np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), d],
#                  [0, 0, 0, 1]])
# tt061 = np.dot(tt01,tt12)
# tt062 = np.dot(tt061,tt23)
# tt063 = np.dot(tt062,tt34)
# tt064 = np.dot(tt063,tt45)
# tt06 = np.dot(tt064,tt56)
# print(tt06)

# r01 = ([[np.cos(theta1), -np.sin(theta1), 0],
#         [np.sin(theta1), np.cos(theta1), 0],
#         [0,0,1]])
# r02 = ([[0,0,-1],[-1,0,0],[0,1,0]])
# R01 = np.matmul(r01,r02)
# r12 = ([[np.cos(theta2), -np.sin(theta2), 0],
#         [np.sin(theta2), np.cos(theta2), 0],
#         [0,0,1]])
# r13 = ([[0,1,0],[1,0,0],[0,0,1]])
# R12 = np.matmul(r12,r13)
# r23 = ([[np.cos(theta3), -np.sin(theta3), 0],
#         [np.sin(theta3), np.cos(theta3), 0],
#         [0,0,1]])
# r24 = ([[0,0,1],[1,0,0],[0,-1,0]])
# R23 = np.matmul(r23,r24)

# R03 = np.dot(R01,R12,R23)
# print(R03)
