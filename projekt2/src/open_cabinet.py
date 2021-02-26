#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult
from math import radians
from math import atan

def deg2rad(deg):
    return float(deg)/180.0*math.pi

def getTorsoRotation(cab_x, cab_y):
    torsoAngle = math.atan2(cab_y, cab_x)
    
    if torsoAngle > 1.55:
        torsoAngle = 1.55
    elif torsoAngle < -1.55:
        torsoAngle = -1.55

    return torsoAngle


def rotateFingersAroundPalm(velma, hand, spread):
    '''
    hand - string: name of hand, either 'left' or 'right'
    spread - integer representing spread of fingers in degrees
    (180 - fingers next to each other, 0 - fingers apart)
    '''    
    curr_q = velma.getHandCurrentConfiguration(hand)
    dest_q = [curr_q[1], curr_q[4], curr_q[6], deg2rad(spread)]
    velma.moveHand(hand, dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    
    if velma.waitForHand(hand) != 0:
        exitError(2)
    rospy.sleep(1)



def moveFingers(velma, hand, angle, spread):
    '''
    hand - string: name of hand, either 'left' or 'right'
    angle - integer representing angle of fingers in degrees
    (140 - fingers closed, 0 - fingers open)
    spread - integer representing spread of fingers in degrees
    (180 - fingers together, 0 - fingers apart)   

    '''    
    curr_q = velma.getHandCurrentConfiguration(hand)
    dest_q = [deg2rad(angle), deg2rad(angle), deg2rad(angle), deg2rad(spread)]
    velma.moveHand(hand, dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    
    if velma.waitForHand(hand) != 0:
        exitError(2)
    rospy.sleep(1)    



def interpolateDoorOpening(velma, totalAngle, degStep):
    #wyliczenie wszystkich stopnie o ktore maja sie obrocic drzwi
    degrees = list(range(degStep, totalAngle, degStep))
    if degrees[-1] != totalAngle:
        degrees.append(totalAngle)
    
    T_B_Zawias = velma.getTf("B", "zawias")

    #obliczenie orignalnej pozycji chwycenia klamki wzgledem zawiasu
    T_zawias_klamka = velma.getTf("zawias", "klamka")
    T_zawias_grasp_org = T_zawias_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.035, +0.07))
    T_zawias_grasp_org = T_zawias_grasp_org * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

    #Lista gotowych TF-ow
    TFs = []

    for deg in degrees:
        #obrocenie zawiasu o okreslony stopien
        T_B_Z_rot = T_B_Zawias * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(deg)), PyKDL.Vector())
        T_B_grasp = T_B_Z_rot * T_zawias_grasp_org
        TFs.append(T_B_grasp)

    return TFs



def exitError(code):
    if code == 0:
        print "OK"
        exit(0)
    print "ERROR:", code
    exit(code)


# starting position
q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
    'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

# right hand ovear the table
q_map_1 = {'torso_0_joint':0, 'right_arm_0_joint':0.1, 'right_arm_1_joint':-1.60,
    'right_arm_2_joint':2.04, 'right_arm_3_joint':1.22, 'right_arm_4_joint':0.41, 'right_arm_5_joint':-1.61,
    'right_arm_6_joint':0.55, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }


if __name__ == "__main__":

    rospy.init_node('open_cab')

    rospy.sleep(0.5)
    br = tf.TransformBroadcaster()

    print "This script opens a cabinet\n"

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

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    #obliczanie docelowej pozycji torsu na podstawie pozycji szafki
    T_B_klamka = velma.getTf("B", "szafka")
    torsoAngle = getTorsoRotation(T_B_klamka.p.x(), T_B_klamka.p.y())
    q_map_starting['torso_0_joint'] = torsoAngle
    q_map_1['torso_0_joint'] = torsoAngle


    #sprawdzenie konfiguracji poczatkowej
    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
        print "This test requires starting pose:"
        print q_map_starting

        print "Moving to the starting position..."
        velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        error = velma.waitForJoint()
        if error != 0:
            print "The action should have ended without error, but the error code is", error
            exitError(6)

        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
            exitError(10)
        

    #przejscie w przestrzeni stawow z prawa reka wyciagnieta do przodu
    print "Moving to position 0 (slowly)."
    velma.moveJoint(q_map_1, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_1, js[1], tolerance=0.1):
        exitError(10)

    rospy.sleep(1.0)


    #ulozenie dloni do chwytania
    print "Rotating fingers to position: together"
    rotateFingersAroundPalm(velma, 'right', 180)

    print "Closing hand a lil' bit"
    moveFingers(velma, 'right', 100, 180)

    
    #zmiana narzedzia
    print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    T_B_Gr = [T_B_Wr*T_Wr_Gr]
    if not velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(18)
    if velma.waitForEffectorRight() != 0:
        exitError(19)
    print "the right tool is now in 'grip' pose"
    rospy.sleep(0.5)


    #przejscie do pozycji przy klamce
    print "move near handle"
    T_B_klamka = velma.getTf("B", "klamka")
    T_B_P1 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.01, +0.1, +0.07))
    T_B_P1 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

    if not velma.moveCartImpRight([T_B_P1], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
    if velma.waitForEffectorRight() != 0:
            exitError(9)


    #przejscie do pozycji, w ktorej Velma jest gotowa zaczac otwierac drzwi
    print "Moving hand closer to handle"
    T_B_P2 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.035, +0.07))
    T_B_P2 = T_B_P2 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

    if not velma.moveCartImpRight([T_B_P2], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(0.4,0.4,0.4), PyKDL.Vector(0.4,0.4,0.4)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.05,0.05,0.05), PyKDL.Vector(0.05,0.05,0.05))):
            exitError(8)
    if velma.waitForEffectorRight() != 0:
            exitError(9)
    rospy.sleep(0.5)


    
    #otworz drzwi o 90 stopnie interpolujac co 10 stopni
    print "Opening door"
    TFs = interpolateDoorOpening(velma, 90, 10)
    print len(TFs)
    for TF in TFs:
        print "move to pos"
        br.sendTransform((TF.p.x(), TF.p.y(), TF.p.z()),
                             TF.M.GetQuaternion(),
                             rospy.Time.now(),
                             "tf",
                             "world")
        if not velma.moveCartImpRight([TF], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(0.4,0.4,0.4), PyKDL.Vector(0.4,0.4,0.4)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.05,0.05,0.05), PyKDL.Vector(0.05,0.05,0.05))):
                exitError(8)
        if velma.waitForEffectorRight() != 0:
                exitError(9)

        rospy.sleep(0.5)
        print "movement done"
        

    #odsuniecie od klamki
    print "Moving away from handle"
    T_B_klamka = velma.getTf("B", "klamka")
    T_B_P1 = T_B_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, +0.12, +0.072))
    T_B_A1 = T_B_P1 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))
    
    if not velma.moveCartImpRight([T_B_A1], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
    if velma.waitForEffectorRight() != 0:
            exitError(9)


    #odejscie od klamki na wieksza odleglosc
    print "Moving far away from handle"
    T_B_A2 = T_B_A1 * PyKDL.Frame(PyKDL.Rotation.RotX(radians(35)), PyKDL.Vector(0, +0.1, -0.1))

    if not velma.moveCartImpRight([T_B_A2], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
    if velma.waitForEffectorRight() != 0:
            exitError(9)


    #powrot do pozycji startowej
    print "Moving to the starting position..."
    velma.moveJoint(q_map_starting, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)

    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.1):
        exitError(10)

    print "Opening fingers"
    moveFingers(velma, 'right', 0, 180)

    print "Rotating fingers to position: apart"
    rotateFingersAroundPalm(velma, 'right', 0)

    
    exitError(0)

