#!/usr/bin/env python3
 
import numpy as np 

# Given position and orientation of the end effector 
px = 350.9
py = 470.5
pz = 485.5
rx = 0.0
ry = 90.0
rz = 0.0

# Link lengths
l1 = 155.5
l2 = 409
l3 = 367
l4 = 124

# Finding the rotational matrix of frame 6 relative 0 using orientation
R_x = np.array([[1,         0,                  0                ],
                [0,         round(np.cos(rx)), round(-np.sin(rx))],
                [0,        round(np.sin(rx)), round(np.cos(rx))  ]])
                

R_y = np.array([[round(np.cos(ry)),    0,      round(np.sin(ry))],
                [0,                    1,      0                ],
                [round(-np.sin(ry)),   0,      round(np.cos(ry))]])
                

R_z = np.array([[round(np.cos(rz)),    round(-np.sin(rz)),    0],
                [round(np.sin(rz)),    round(np.cos(rz)),     0],
                [0,                     0,                    1]])
                
rot_mat_06= np.array(np.dot(R_z, np.dot( R_y, R_x )))


# Finding the wrist centre point of the manipulator
x = px - (l4*rot_mat_06[0,2])
y = py - (l4*rot_mat_06[1,2])
z = pz - (l4*rot_mat_06[2,2])

# calculating the first three angles 
r2 = (z-l1)
r1 = (((x**2)+(y**2))**0.5)
r3 = (((r1**2)+(r2**2))**0.5)
a1 = (((l3**2)-(l2**2)-(r3**2))/(-2*l2*r3))
A1 = np.degrees(np.arccos(a1))
A2 = np.degrees(np.arctan2(r2,r1))
a3 = (((r3**2)-(l2**2)-(l3**2))/(-2*l2*l3))
A3 = np.degrees(np.arccos(a3))
theta_1 = np.degrees(np.arctan2(y,x))
print(f'Theta 1 = {theta_1} degrees\n')
theta_2 =(A2-A1)
print(f'Theta 2 = {theta_2} degrees\n')
theta_3 = (180-A3)
print(f'Theta 3 = {theta_3} degrees\n')

# Finding the rotational matrix of frame 3 relative to frame 0
rot_mat_0_3 = np.array([[-np.sin(theta_2), 0.0, np.cos(theta_2)],
                        [np.cos(theta_2), 0.0, np.sin(theta_2)],
                        [0.0, 1.0, 0.0]])
 
# Calculate the inverse rotation matrix
inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)
 
# Calculate the rotational matrix of frame 6 relative to frame 3
rot_mat_3_6 = np.matmul(inv_rot_mat_0_3,rot_mat_06)
print(f'rot_mat_3_6 = {rot_mat_3_6}')
 
theta_5 = np.degrees(np.arccos(rot_mat_3_6[2, 2]))
print(f'Theta 5 = {theta_5} degrees\n') 

theta_6 = np.degrees(np.arccos(rot_mat_3_6[2, 0] / -np.sin(theta_5)))
print(f'Theta 6 = {theta_6}  degrees\n')
 
theta_4 = np.degrees(np.arccos(rot_mat_3_6[1,2] / np.sin(theta_5)))
# theta4 = (np.arctan2(r23,r13))
print(f'Theta 4 = {theta_4}  degrees\n')
 
# Check that the angles we calculated result in a valid rotation matrix
r11 = -np.sin(theta_4) * np.cos(theta_5) * np.cos(theta_6) - np.cos(theta_4) * np.sin(theta_6)
r12 = np.sin(theta_4) * np.cos(theta_5) * np.sin(theta_6) - np.cos(theta_4) * np.cos(theta_6)
r13 = -np.sin(theta_4) * np.sin(theta_5)
r21 = np.cos(theta_4) * np.cos(theta_5) * np.cos(theta_6) - np.sin(theta_4) * np.sin(theta_6)
r22 = -np.cos(theta_4) * np.cos(theta_5) * np.sin(theta_6) - np.sin(theta_4) * np.cos(theta_6)
r23 = np.cos(theta_4) * np.sin(theta_5)
r31 = -np.sin(theta_5) * np.cos(theta_6)
r32 = np.sin(theta_5) * np.sin(theta_6)
r33 = np.cos(theta_5)

check_rot_mat_3_6 = np.array([[r11, r12, r13],
                              [r21, r22, r23],
                              [r31, r32, r33]])
 
# Original matrix
print(f'\nrot_mat_3_6 = {rot_mat_3_6}')
 
# Check Matrix
print(f'\ncheck_rot_mat_3_6 = {check_rot_mat_3_6}\n')
 
# Return if Original Matrix == Check Matrix
rot_minus_check_3_6 = rot_mat_3_6.round() - check_rot_mat_3_6.round()
zero_matrix = np.array([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])
matrices_are_equal = np.array_equal(rot_minus_check_3_6, zero_matrix)
 
# Determine if the solution is valid or not
# If the solution is valid, that means the end effector of the robotic 
# arm can reach that target location.
if (matrices_are_equal):
  valid_matrix = "Yes"
else:
  valid_matrix = "No" 
print(f'Is this solution valid?\n{valid_matrix}\n')