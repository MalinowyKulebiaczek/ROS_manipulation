#!/usr/bin/env python2

## Runs test for motions in cart_imp mode.
# @ingroup integration_tests
# @file test_cimp_pose.py
# @namespace scripts.test_cimp_pose Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
from math import radians
from math import sqrt

if __name__ == "__main__":
    rospy.init_node('grasp_jar')

    rospy.sleep(0.5)
    br = tf.TransformBroadcaster()

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    

    T_B_S = velma.getTf("B", "sloik")
    T_B_T1 = velma.getTf("B", "stol1")
    T_B_T2 = velma.getTf("B", "stol2")

    x_distance_sloik_stol1 = T_B_S.p.x() - T_B_T1.p.x()
    y_distance_sloik_stol1 = T_B_S.p.y() - T_B_T1.p.y()

    x_distance_sloik_stol2=T_B_S.p.x()-T_B_T2.p.x()
    y_distance_sloik_stol2=T_B_S.p.y()-T_B_T2.p.y()

    distance_sloik_stol1 = sqrt(x_distance_sloik_stol1**2 + y_distance_sloik_stol1**2) 
    distance_sloik_stol2 = sqrt(x_distance_sloik_stol2**2 + y_distance_sloik_stol2**2)

    if distance_sloik_stol1 <= distance_sloik_stol2:
        print "Object is on stol1"
        T_B_T = T_B_T2 #docelowym jest stol2 
    else:
        print "Object is on stol2"
        T_B_T = T_B_T1 #docelowym jest stol1

    if T_B_S.p.y() < 0:
        print "object is on the right"
        r = -1			#zmienna sluzaca do obracania pozycji jesli sloik jest po prawej stronie
    	o = 180 		#ilosc stopni o jaka trzeba obrocic pewne
    	object_pos = 'right'
    else:
        print "Object is on the left"
        r = 1
        o = 0
        object_pos = 'left'

    while not rospy.is_shutdown():
        
        T_B_P1_pos = PyKDL.Vector(T_B_S.p.x() - 0.24, T_B_S.p.y(), T_B_S.p.z() + 0.1)
        T_B_P1 = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0), T_B_P1_pos)
        T_B_P1 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(180-o)))
        br.sendTransform((T_B_P1.p.x(), T_B_P1.p.y(), T_B_P1.p.z()),
                         T_B_P1.M.GetQuaternion(),
                         rospy.Time.now(),
                         "pregrasp_1",
                         "world")

        if object_pos == 'left':
        	T_B_P2 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(-30)), PyKDL.Vector(0.1, -0.15, 0.2))
        	T_B_P3_pos = PyKDL.Vector(T_B_T.p.x()-0.30, T_B_T.p.y()+0.43, T_B_P1.p.z()) #x i y - stol docelowy. z - wysokosc sloika zanim zostal chwycony
        	T_B_P3_rot = (T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians((-80)*r)))).M
        	T_B_P3 = PyKDL.Frame(T_B_P3_rot, T_B_P3_pos)
        	T_B_P4 = T_B_P3 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(25)), PyKDL.Vector(0.25, 0, 0.25))

			#T_B_P2 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians((-30)*r)), PyKDL.Vector(0.1, (-0.15)*r, 0.2))
        	#T_B_P3_pos = PyKDL.Vector(T_B_T.p.x()-0.30, T_B_T.p.y()+0.43*r, T_B_P1.p.z()) #x i y - stol docelowy. z - wysokosc sloika zanim zostal chwycony
        	#T_B_P3_rot = (T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians((-80)*r)))).M
        	#T_B_P3 = PyKDL.Frame(T_B_P3_rot, T_B_P3_pos)
        	#T_B_P4 = T_B_P3 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(25*r)), PyKDL.Vector(0.25, 0, 0.25))

    	elif object_pos =='right':
    		T_B_P2 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(30)), PyKDL.Vector(-0.1, -0.15, 0.2))
        	T_B_P3_pos = PyKDL.Vector(T_B_T.p.x()-0.30, T_B_T.p.y()-0.43, T_B_P1.p.z()) #x i y - stol docelowy. z - wysokosc sloika zanim zostal chwycony
        	T_B_P3_rot = (T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(80)))).M
        	T_B_P3 = PyKDL.Frame(T_B_P3_rot, T_B_P3_pos)
        	T_B_P4 = T_B_P3 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(-25)), PyKDL.Vector(-0.25, 0, 0.25))

        

        
        br.sendTransform((T_B_P2.p.x(), T_B_P2.p.y(), T_B_P2.p.z()),
                         T_B_P2.M.GetQuaternion(),
                         rospy.Time.now(),
                         "pregrasp_2",
                         "world")
        
        
        br.sendTransform((T_B_P3.p.x(), T_B_P3.p.y(), T_B_P3.p.z()),
                         T_B_P3.M.GetQuaternion(),
                         rospy.Time.now(),
                         "predrop_1",
                         "world")

        
        br.sendTransform((T_B_P4.p.x(), T_B_P4.p.y(), T_B_P4.p.z()),
                         T_B_P4.M.GetQuaternion(),
                         rospy.Time.now(),
                         "afterdrop_1",
                         "world")

        rospy.sleep(0.1)

 