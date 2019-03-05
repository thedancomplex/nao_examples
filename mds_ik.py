#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2015, Daniel M. Lofaro
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



#import mds_ach as ha
import ach
import sys
import time
import math
import numpy as np
from numpy.linalg import inv
from ctypes import *
#import mds_ik_include as ike
import copy


#threshold = .025
threshold = .0025
## l1 = .2145   # center of body to shoulder
## l2 = .17914  # shoulder to eblow
## l3 = .18159  # elbow to wrist roll


UpperArmLength  =105.0
ElbowOffsetY    =15.0
LowerArmLength  =55.95
HandOffsetX 	=57.75
ShoulderOffsetY =98.0
HandOffsetZ     =12.31

l1 = ShoulderOffsetY#0.2455100   # center of body to shoulder
l2 = UpperArmLength#0.2825750   # shoulder to eblow
l3 = LowerArmLength#0.3127375   # elbow to wrist roll
l4 = HandOffsetX#0.0635      # distance to middle of hand
alpha = .5
dT1 = .001
dT2 = .001
dT3 = .001

#open ach channel for coodinates being sent from file
### k = ach.Channel(ike.CONTROLLER_REF_NAME)
#coordinates = ike.CONTROLLER_REF()
#coordinates.x = -0.25
#coordinates.y = -0.20
#coordinates.z =  0.20

# feed-forward will now be refered to as "state"
#state = ha.MDS_REF()

# feed-back will now be refered to as "ref"
#ref = ha.MDS_REF()

#bend elbow to start to force jacobian to solve in righ direction
#avoid singularity
#ref.joint[ha.REB].ref = -math.pi/6
#ref.joint[ha.LEB].ref = -math.pi/6

### r.put(ref)
### time.sleep(3)

# rotation matrix definition (abotu x, y, or z)
def_x = 1
def_y = 2
def_z = 3


def getRotMatrix(t, d):
    if d == def_x:
        return np.matrix([[1.0, 0.0, 0.0], [ 0.0, np.cos(t), -np.sin(t)], [ 0.0, np.sin(t), np.cos(t)]])
    elif d == def_y:
        return np.matrix([[np.cos(t), 0.0, np.sin(t)], [ 0.0, 1.0, 0.0], [ -np.sin(t), 0.0, np.cos(t)]])
    elif d == def_z:
        return np.matrix([[np.cos(t), -np.sin(t), 0.0], [ np.sin(t), np.cos(t), 0.0], [ 0.0, 0.0, 1.0]])
    else:
        return np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

def getTransMatrix(x,y,z):
    return np.matrix([[x], [y], [z]])

def getTfMatrix(R,T):
    Ap = np.hstack([R,T])
    return np.vstack([Ap,[0.0, 0.0, 0.0, 1.0]])

def getA(a):
#    a = [theta  , Tx  , Ty        , Tz  , def_xyz]
    R = getRotMatrix(a[0], a[4])
    T = getTransMatrix(a[1],a[2],a[3])
    A  = getTfMatrix(R,T)
    return A

def getFkArm(a, arm):
  m = 1.0
  if arm == 'right':
    m = -1.0
  elif arm == 'left':
    m = 1.0
  else:
    return -1

  #       theta        , Tx       , Ty          , Tz         , def_xyz
  LSP = [ a[0]         , 0.0      , m*l1        , 0.0        , def_y ]
  LSR = [ a[1]         , 0.0      , 0.0         , 0.0        , def_x ]
  LSY = [ a[2]         , 0.0      , 0.0         , -l2        , def_z ]
  LEB = [ a[3]         , 0.0      , 0.0         , 0.0        , def_y ]
  LWY = [ a[4]         , 0.0      , 0.0         , -l3        , def_z ]
  LWR = [ a[5]         , 0.0      , 0.0         , 0.0        , def_x ]
  LEN = [ 0.0          , 0.0      , 0.0         , -l4        , def_x ]
  # note rolls seem to be oppisit (-)

  A1 = getA(LSP)
  A2 = getA(LSR)
  A3 = getA(LSY)
  A4 = getA(LEB)
  A5 = getA(LWY)
  A6 = getA(LWR)
  A7 = getA(LEN)
  #print(A7)
  A = np.dot(A1,A2)
  #A = np.dot(A, A2)
  A = np.dot(A, A3)
  A = np.dot(A, A4)
  A = np.dot(A, A5)
  A = np.dot(A, A6)
  A = np.dot(A, A7)

#dan
#  A = np.dot(A7,A6)
#  A = np.dot(A5, A)
#  A = np.dot(A4, A)
#  A = np.dot(A3, A)
#  A = np.dot(A2, A)
#  A = np.dot(A1, A)

  return A


def getJacobian6x3(d0,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm

  # Jacobian for xyz and 6 dof
  xyz = 3
  dof = 6
  J = np.zeros((xyz,dof))
  # J[ row, col ]
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  for i in range(xyz):
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        J[i,j] = (A1[i,3] - A0[i,3])/dt
  return J



def getJacobian(d0,order,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm
  # order is the order of the desired joints [x, y, z, t_x, t_y, t_z]
  #                                          [x, y, z, t_x]
  #                                          [x, y, z, t_z]
  #                                          [x, y, z, t_y, t_z]
  #                                          Etc.

  # Jacobian or size dof x order
  xyz = len(order)
  dof = len(d0)
  J = np.zeros((xyz,dof))
  # J[ row, col ]
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  for i in range(xyz):
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if order[i] == 'p_x':
          J[i,j] = (A1[0,3] - A0[0,3])/dt
        if order[i] == 'p_y':
          J[i,j] = (A1[1,3] - A0[1,3])/dt
        if order[i] == 'p_z':
          J[i,j] = (A1[2,3] - A0[2,3])/dt
        if order[i] == 't_x':
          a0 = getRot(A0,'x')
          a1 = getRot(A1,'x')
          J[i,j] = (a1-a0)/dt
        if order[i] == 't_y':
          a0 = getRot(A0,'y')
          a1 = getRot(A1,'y')
          J[i,j] = (a1-a0)/dt
        if order[i] == 't_z':
          a0 = getRot(A0,'z')
          a1 = getRot(A1,'z')
          J[i,j] = (a1-a0)/dt
  return J
def getJacobian6x6(d0,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm

  # Jacobian for xyz and 6 dof
  xyz = 6
  dof = 6
  J = np.zeros((xyz,dof))
  # J[ row, col ]
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  for i in range(xyz):
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if i <= 2:
          J[i,j] = (A1[i,3] - A0[i,3])/dt
        else:
          if i == 3:
             a0 = getRot(A0,'x')
             a1 = getRot(A1,'x')
             J[i,j] = (a1-a0)/dt
          if i == 4:
             a0 = getRot(A0,'y')
             a1 = getRot(A1,'y')
             J[i,j] = (a1-a0)/dt
          if i == 5:
             a0 = getRot(A0,'z')
             a1 = getRot(A1,'z')
             J[i,j] = (a1-a0)/dt
  return J


def getRot(a,ax):
  if ax == 'x':
    return np.arctan2(a[2,1],a[2,2])
  elif ax == 'y':
    return np.arctan2(-a[2,0],np.sqrt(a[2,1]*a[2,1]+a[2,2]*a[2,2]))
  elif ax == 'z':
    return np.arctan2(a[1,0], a[0,0])
  else:
    return -1


def getDist2End(eff_current, eff_end):
  eff_vector = eff_end - eff_current # vector to end point

  eff_dist_to_end = np.sqrt(eff_vector[0]*eff_vector[0] +
                            eff_vector[1]*eff_vector[1] +
                            eff_vector[2]*eff_vector[2])
  return eff_dist_to_end

def getDist2End2(eff_current, eff_end):
  eff_vector = eff_end - eff_current # vector to end point

  vsum = 0.0
  for i in range(len(eff_vector)):
    vsum = eff_vector[i]*eff_vector[i] + vsum

  eff_dist_to_end = np.sqrt(vsum)
  return eff_dist_to_end

#print A

def getIK3dof(eff_joint_space_current, eff_end, arm):
 # eff_joint_space_current = [theta_SP, SR, RY, EP, WY, RR]
 # eff_end = desired end effector positon = [x,y,z]
 # arm = 'left' or 'right' arm to solve for
 A = getFkArm(eff_joint_space_current,arm)
## eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
 eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
# ef  = np.array([0.2, 0.2, -0.1])
## eff_end  = np.array([-0.14583 , 0.74283, 0.13834])

 eff_delta_theta = 0.01 # change in goal in meters
 eff_delta_xyz = 0.01 # change in goal in meters
 eff_err_max = 0.01

 # distance to end point
 eff_dist_to_end = getDist2End(eff_current, eff_end)

 while (eff_err_max < eff_dist_to_end):
  # jacobian of the eff_next_point
  J = getJacobian6x3(eff_joint_space_current, eff_delta_theta, arm)

  # compute inverse of jacobian
  Ji = np.linalg.pinv(J)   # inverse

  # linear interpolation to find next point
  eff_vector = eff_end - eff_current # vector to end point
  eff_dist_to_end = getDist2End(eff_current, eff_end)
  d_eff = eff_vector/eff_dist_to_end * eff_delta_xyz
 # print d_eff
  # compute change in DOFs d_theta = Ji * d_eff
  d_theta = np.dot(Ji,d_eff)

  # apply changes to dofs
  eff_joint_space_current = eff_joint_space_current + d_theta

  # distance to end point
  A = getFkArm(eff_joint_space_current,arm)
  eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
  eff_dist_to_end = getDist2End(eff_current, eff_end)

#  print eff_dist_to_end
 return eff_joint_space_current

def getPosCurrentFromOrder(A,order):
  J = np.zeros(len(order))
  for i in range(len(order)):
        if order[i] == 'p_x':
          J[i] = A[0,3]
        if order[i] == 'p_y':
          J[i] = A[1,3]
        if order[i] == 'p_z':
          J[i] = A[2,3]
        if order[i] == 't_x':
          a1 = getRot(A,'x')
          J[i] = a1
        if order[i] == 't_y':
          a1 = getRot(A,'y')
          J[i] = a1
        if order[i] == 't_z':
          a1 = getRot(A,'z')
          J[i] = a1
  return J


def getIK(eff_joint_space_current, eff_end, order, arm, err=None, itr=None):
 # eff_joint_space_current = [theta_SP, SR, RY, EP, WY, RR]
 # eff_end = desired end effector positon = [x,y,z,theta_x,theta_y,theta_z]
 # order is the order of the desired joints [p_x, p_y, p_z, t_x, t_y, t_z]
 #                                          [x, y, z, t_x]
 #                                          [x, y, z, t_z]
 #                                          [x, y, z, t_y, t_z]
 # arm = 'left' or 'right' arm to solve for
 # err (optional) = error for delta_theta, delta_pos (xyz) and maximum error
 # itr (optional) = number of max iterations
 eff_joint_space_orig = copy.deepcopy(eff_joint_space_current)
 eff_delta_theta = 0.01 # change in goal in meters
 eff_delta_xyz = 0.01 # change in goal in meters
 eff_err_max = 0.01

 if (err is not None):
  eff_delta_theta = err[0] # change in goal in rad
  eff_delta_xyz = err[1] # change in goal in meters
  eff_err_max = err[2]  # max linear error of eef

 A = getFkArm(eff_joint_space_current,arm)
## eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
 eff_current = getPosCurrentFromOrder(A,order)
# ef  = np.array([0.2, 0.2, -0.1])
## eff_end  = np.array([-0.14583 , 0.74283, 0.13834])

 # distance to end point
 eff_dist_to_end = getDist2End2(eff_current, eff_end)
 itr_i = 0
 while (eff_err_max < eff_dist_to_end):
  # jacobian of the eff_next_point
  J = getJacobian(eff_joint_space_current,order,eff_delta_theta,arm)
##  J = getJacobian6x6(eff_joint_space_current, eff_delta_theta, arm)

  # compute inverse of jacobian
  Ji = np.linalg.pinv(J)   # inverse

  # linear interpolation to find next point
  eff_vector = eff_end - eff_current # vector to end point
  eff_dist_to_end = getDist2End2(eff_current, eff_end)
  d_eff = eff_vector/eff_dist_to_end * eff_delta_xyz
 # print d_eff
  # compute change in DOFs d_theta = Ji * d_eff
  d_theta = np.dot(Ji,d_eff)

  # apply changes to dofs
  eff_joint_space_current = eff_joint_space_current + d_theta

  # Check for over joint range
  m = 1.0
  if arm == 'left':
     m = 1.0
  elif arm == 'right':
     m = -1.0

  if eff_joint_space_current[0] > np.pi:     #xSP max
     eff_joint_space_current[0] = np.pi
  if eff_joint_space_current[0] < -np.pi:    #xSP min
     eff_joint_space_current[0] = -np.pi
  if arm == 'left':
    if eff_joint_space_current[1] < 0.0:     #xSR towards body
       eff_joint_space_current[1] = 0.0
    if eff_joint_space_current[1] > np.pi:
       eff_joint_space_current[1] = np.pi
  if arm == 'right':
    if eff_joint_space_current[1] > 0.0001:     #xSR towards body
       eff_joint_space_current[1] = 0.0001
    if eff_joint_space_current[1] < -np.pi:
       eff_joint_space_current[1] = -np.pi
  if eff_joint_space_current[2] > np.pi:     #xSY max
     eff_joint_space_current[2] = np.pi
  if eff_joint_space_current[2] < -np.pi:    #xSY min
     eff_joint_space_current[2] = -np.pi
  if eff_joint_space_current[3] > 0.0:     #xeP max
     eff_joint_space_current[3] = 0.0
  if eff_joint_space_current[3] < -2.4:    #xeP min
     eff_joint_space_current[3] = -2.4
  if eff_joint_space_current[4] > np.pi:     #xWY max
     eff_joint_space_current[4] = np.pi
  if eff_joint_space_current[4] < -np.pi:    #xWY min
     eff_joint_space_current[4] = -np.pi
  if eff_joint_space_current[5] > np.pi/2.0 * 0.3:     #xWR max
     eff_joint_space_current[5] = np.pi/2.0 * 0.3
  if eff_joint_space_current[5] < -np.pi/2.0 * 0.3:    #xWR min
     eff_joint_space_current[5] = -np.pi/2.0 * 0.3

  # distance to end point
  A = getFkArm(eff_joint_space_current,arm)
  eff_current = getPosCurrentFromOrder(A,order)
  eff_dist_to_end = getDist2End2(eff_current, eff_end)
  print itr_i, ' - ', eff_dist_to_end
  if (itr_i >= itr):
     return (eff_joint_space_current, -1)
#     return (eff_joint_space_orig, -1)
  else:
     itr_i = itr_i+1
##  print eff_dist_to_end
 return (eff_joint_space_current, 0)


def getIK6dof(eff_joint_space_current, eff_end, arm):
 # eff_joint_space_current = [theta_SP, SR, RY, EP, WY, RR]
 # eff_end = desired end effector positon = [x,y,z,theta_x,theta_y,theta_z]
 # arm = 'left' or 'right' arm to solve for
 A = getFkArm(eff_joint_space_current,arm)
## eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
 eff_current = np.array([ A[0,3], A[1,3], A[2,3], getRot(A,'x'), getRot(A,'y'), getRot(A,'z')])
# ef  = np.array([0.2, 0.2, -0.1])
## eff_end  = np.array([-0.14583 , 0.74283, 0.13834])

 eff_delta_theta = 0.01 # change in goal in meters
 eff_delta_xyz = 0.01 # change in goal in meters
 eff_err_max = 0.01

 # distance to end point
 eff_dist_to_end = getDist2End(eff_current, eff_end)

 while (eff_err_max < eff_dist_to_end):
  # jacobian of the eff_next_point
  J = getJacobian6x6(eff_joint_space_current, eff_delta_theta, arm)

  # compute inverse of jacobian
  Ji = np.linalg.inv(J)   # inverse

  # linear interpolation to find next point
  eff_vector = eff_end - eff_current # vector to end point
  eff_dist_to_end = getDist2End(eff_current, eff_end)
  d_eff = eff_vector/eff_dist_to_end * eff_delta_xyz
 # print d_eff
  # compute change in DOFs d_theta = Ji * d_eff
  d_theta = np.dot(Ji,d_eff)

  # apply changes to dofs
  eff_joint_space_current = eff_joint_space_current + d_theta

  # distance to end point
  A = getFkArm(eff_joint_space_current,arm)
  eff_current = np.array([ A[0,3], A[1,3], A[2,3], getRot(A,'x'), getRot(A,'y'), getRot(A,'z')])
  eff_dist_to_end = getDist2End(eff_current, eff_end)

##  print eff_dist_to_end
 return eff_joint_space_current
print(getFkArm([np.radians(70),np.radians(10),np.radians(10),np.radians(10),np.radians(0),0],'right'))
