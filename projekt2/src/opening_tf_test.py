#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult
from math import radians
from math import atan, sin, cos

def interpolateDoorOpening(velma, totalAngle, degStep):
    #wyliczenie wszystkich stopnie o ktore maja sie obrocic drzwi
    degrees = list(range(0, totalAngle, degStep))
    if degrees[-1] != totalAngle:
        degrees.append(totalAngle)
    
    T_B_Zawias = velma.getTf("B", "zawias")

    #obliczenie orignalnej pozycji chwycenia klamki wzgledem zawiasu
    T_zawias_klamka = velma.getTf("zawias", "klamka")
    T_zawias_grasp_org = T_zawias_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.045, +0.07))
    T_zawias_grasp_org = T_zawias_grasp_org * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

    #Lista gotowych TF-ow
    TFs = []

    for deg in degrees:
        #obrocenie zawiasu o okreslony stopien
        T_B_Z_rot = T_B_Zawias * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(deg)), PyKDL.Vector())
        T_B_grasp = T_B_Z_rot * T_zawias_grasp_org
        TFs.append(T_B_grasp)

    return TFs

if __name__ == "__main__":
    # define some configurations

    rospy.init_node('rot2')

    rospy.sleep(0.5)
    br = tf.TransformBroadcaster()


    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)

    rospy.sleep(0.5)

    TFs = interpolateDoorOpening(velma, 90, 10)

    while not rospy.is_shutdown():

        for TF in TFs:
            br.sendTransform((TF.p.x(), TF.p.y(), TF.p.z()),
                             TF.M.GetQuaternion(),
                             rospy.Time.now(),
                             "klamka2",
                             "world")
            rospy.sleep(0.5)

        rospy.sleep(0.1)