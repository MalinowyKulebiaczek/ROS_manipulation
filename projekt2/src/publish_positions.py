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
from math import atan

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

    

    T_B_klamka = velma.getTf("B", "klamka")
    
    T_B_Wr = velma.getTf("B", "Wr")
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    #T_[T_B_Wr*T_Wr_Gr]
    while not rospy.is_shutdown():
        #T_B_P1_rot = atan(T_B_klamka.p.y()/T_B_klamka.p.x()) #skierowanie ukladu w kierunku robota (tylko w osi z)
        #T_B_P1_rot += radians(-30) #lekkie obrocenie go
        #rot = PyKDL.Rotation()
        #rot.DoRotZ(T_B_P1_rot)


        #T_B_P1_pos = PyKDL.Vector(T_B_klamka.p.x(), T_B_klamka.p.y()-0.1, T_B_klamka.p.z())
        #T_B_P1 = PyKDL.Frame(rot, T_B_P1_pos)
        T_B_P1 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.01, +0.1, 0))
        T_B_P1 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))
        br.sendTransform((T_B_P1.p.x(), T_B_P1.p.y(), T_B_P1.p.z()),
                         T_B_P1.M.GetQuaternion(),
                         rospy.Time.now(),
                         "pregrasp_1",
                         "world")

        T_B_P2 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, +0.01, 0.0))
        T_B_P2 = T_B_P2 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))
        br.sendTransform((T_B_P2.p.x(), T_B_P2.p.y(), T_B_P2.p.z()),
                         T_B_P2.M.GetQuaternion(),
                         rospy.Time.now(),
                         "pregrasp_2",
                         "world")

        T_B_A1 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(0)), PyKDL.Vector(0, -0.1, -0.1))
        br.sendTransform((T_B_A1.p.x(), T_B_A1.p.y(), T_B_A1.p.z()),
                         T_B_A1.M.GetQuaternion(),
                         rospy.Time.now(),
                         "aftergrasp_1",
                         "world")

        rospy.sleep(0.1)

 