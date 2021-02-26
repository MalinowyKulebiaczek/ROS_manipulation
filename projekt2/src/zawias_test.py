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
    degrees = list(range(degStep, totalAngle, degStep))
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

    rospy.init_node('rot')

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

    interpolateDoorOpening(velma, 90, 10)

    rot_angle = radians(30)

    while not rospy.is_shutdown():

        T_B_klamka = velma.getTf("B", "klamka")
        T_B_P1 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.01, +0.1, +0.07))
        T_B_P1 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

        #br.sendTransform((T_B_P1.p.x(), T_B_P1.p.y(), T_B_P1.p.z()),
        #                     T_B_P1.M.GetQuaternion(),
        #                     rospy.Time.now(),
        #                     "1",
        #                     "world")

        T_B_P2 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.035, +0.07))
        T_B_P2 = T_B_P2 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

        br.sendTransform((T_B_P2.p.x(), T_B_P2.p.y(), T_B_P2.p.z()),
                             T_B_P1.M.GetQuaternion(),
                             rospy.Time.now(),
                             "grasp_pos",
                             "world")


        T_B_Z1= velma.getTf("B", "zawias")
        T_B_Z2 = T_B_Z1 * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(30)), PyKDL.Vector())

        br.sendTransform((T_B_Z2.p.x(), T_B_Z2.p.y(), T_B_Z2.p.z()),
                             T_B_Z2.M.GetQuaternion(),
                             rospy.Time.now(),
                             "Z2",
                             "world")

        T_zawias_klamka = velma.getTf("zawias", "klamka")

        T_zawias_klamka2 = T_zawias_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.045, +0.07))
        T_zawias_klamka2 = T_zawias_klamka2 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

        T_B_klamka2 = T_B_Z1 * T_zawias_klamka2

        br.sendTransform((T_B_klamka2.p.x(), T_B_klamka2.p.y(), T_B_klamka2.p.z()),
                             T_B_klamka2.M.GetQuaternion(),
                             rospy.Time.now(),
                             "klamka 2",
                             "world")

        T_B_klamka3 = T_B_Z2 * T_zawias_klamka2

        br.sendTransform((T_B_klamka3.p.x(), T_B_klamka3.p.y(), T_B_klamka3.p.z()),
                             T_B_klamka3.M.GetQuaternion(),
                             rospy.Time.now(),
                             "klamka 3",
                             "world")


        rospy.sleep(0.1)