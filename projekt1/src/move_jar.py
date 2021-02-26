#!/usr/bin/env python2


#Plik zrobiony na podstawie test_jimp.py

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import PyKDL

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

from math import radians, atan

def deg2rad(deg):
    return float(deg)/180.0*math.pi

def openHand(velma, hand):
    '''
    hand - string "left" or "right"
    '''
    print "Opening left grippers"
    '''
    dest_q = [deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(180)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    
    '''
    dest_q = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)]
    if hand == 'left':
        velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 1000000, hold=False)
        if velma.waitForHandLeft() != 0:
            exitError(2)
    elif hand == 'right':
        velma.moveHandRight(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 1000000, hold=False)
        if velma.waitForHandRight() != 0:
            exitError(2)
    else:
        print "ERROR: wrong hand specifier"
        return

def openBothHands(velma):

    print "Opening grippers"
    '''
    #otwarcie palcow
    dest_q = [deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(180)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    velma.moveHandRight(dest_q, [1.25, 1.25, 1.25, 1.25], [4000,4000,4000,4000], 1000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    if velma.waitForHandRight() != 0:
        exitError(2)
    '''
    #rozsuniecie
    dest_q = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    velma.moveHandRight(dest_q, [1.25, 1.25, 1.25, 1.25], [4000,4000,4000,4000], 1000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    if velma.waitForHandRight() != 0:
        exitError(2)
    
def closeBothHands(velma):
    print "Hiding grippers"
    
    #przesuniecie palcow blisko siebie
    dest_q = [deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(180)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    velma.moveHandRight(dest_q, [1.25, 1.25, 1.25, 1.25], [4000,4000,4000,4000], 1000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    if velma.waitForHandRight() != 0:
        exitError(2)
#    rospy.sleep(3)

    #schowanie palcow
    dest_q = [deg2rad(140), deg2rad(140), deg2rad(140), deg2rad(180)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 1000, hold=False)
    velma.moveHandRight(dest_q, [1.25, 1.25, 1.25, 1.25], [4000,4000,4000,4000], 1000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    if velma.waitForHandRight() != 0:
        exitError(2)

def graspJar(velma, init_deg, hand):
    print "Closing grippers"
    dest_q = [deg2rad(init_deg), deg2rad(init_deg), deg2rad(init_deg), deg2rad(0)]
    if hand == 'left':
        velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
        if velma.waitForHandLeft() != 0:
                exitError(2)
        currentHandConfiguration = velma.getHandLeftCurrentConfiguration()
        handConfInRad = [deg2rad(currentHandConfiguration[2]), deg2rad(currentHandConfiguration[5]), deg2rad(currentHandConfiguration[7])]
        #print"current conf:{}".format(velma.getHandLeftCurrentConfiguration())
        #print"dest    conf:{}".format(dest_q)
        #print"confinrad conf:{}".format(leftHandConfInRad)
    elif hand == 'right':
        velma.moveHandRight(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
        if velma.waitForHandRight() != 0:
                exitError(2)
        currentHandConfiguration = velma.getHandRightCurrentConfiguration()
        handConfInRad = [deg2rad(currentHandConfiguration[2]), deg2rad(currentHandConfiguration[5]), deg2rad(currentHandConfiguration[7])]
    else:
        print "ERROR: wrong hand specifier"
        return
    
    i=0
    while(isHandConfigurationClose(currentHandConfiguration, dest_q, tolerance=0.1)):
        
        i += 2
        dest_q = [deg2rad(init_deg+i), deg2rad(init_deg+i), deg2rad(init_deg+i), deg2rad(0)]

        if hand == 'left':
            velma.moveHandLeft(dest_q, [1, 1, 1, 1], [4000, 4000, 4000, 4000], 1000, hold=False)
            if velma.waitForHandLeft() != 0:
                exitError(2)
            currentHandConfiguration = velma.getHandLeftCurrentConfiguration()
        elif hand == 'right':
            velma.moveHandRight(dest_q, [1, 1, 1, 1], [4000, 4000, 4000, 4000], 1000, hold=False)
            if velma.waitForHandRight() != 0:
                exitError(2)
            currentHandConfiguration = velma.getHandRightCurrentConfiguration()

        print"closing angle in deg: {}".format(i+init_deg)
        print"current conf:{}".format(currentHandConfiguration)
        print"dest    conf:{}".format(dest_q)

def moveHandToPosCart(velma, tf_name, hand):
    '''
    tf_name - string of tf we want our hand to move to
    hand - string "left" or "right"
    '''

    T_B_Trd = velma.getTf("B", tf_name)
    
    if hand == 'left':
        if not velma.moveCartImpLeft([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if velma.waitForEffectorLeft() != 0:
            exitError(9)
    elif hand == 'right':
        if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
            exitError(9)
    rospy.sleep(0.5)

    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", tf_name), 1.0)
    print T_B_T_diff
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(10)


if __name__ == "__main__":
    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_above_tables = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-0.65,  'left_arm_1_joint':0.65,
        'right_arm_2_joint':1.65,   'left_arm_2_joint':-1.65,
        'right_arm_3_joint':1.3,    'left_arm_3_joint':-1.3,
        'right_arm_4_joint':1.56,   'left_arm_4_joint':-1.56,
        'right_arm_5_joint':-1.56,  'left_arm_5_joint':1.56,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_before_drop = {'torso_0_joint':-0.7,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_before_drop_left = {'torso_0_joint':-0.7,
        'right_arm_0_joint':-0.1,   'left_arm_0_joint':-0.4,
        'right_arm_1_joint':-0.64,  'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.65,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':1.26,   'left_arm_3_joint':-1.25,
        'right_arm_4_joint':1.48,   'left_arm_4_joint':-0.5,
        'right_arm_5_joint':-1.34,  'left_arm_5_joint':1.73,
        'right_arm_6_joint':-0.24,  'left_arm_6_joint':-0.3 }

    q_map_before_drop_right = {'torso_0_joint':0.71,
        'right_arm_0_joint':0.0,   'left_arm_0_joint':-0.15,
        'right_arm_1_joint':-1.55,   'left_arm_1_joint':1.7,
        'right_arm_2_joint':1.55,   'left_arm_2_joint':1.84,
        'right_arm_3_joint':1.4,   'left_arm_3_joint':1.25,
        'right_arm_4_joint':0.5,   'left_arm_4_joint':-0.15,
        'right_arm_5_joint':-1.7,   'left_arm_5_joint':1.27,
        'right_arm_6_joint':1,    'left_arm_6_joint':-0.11 }


    rospy.init_node('grasp_and_lift')

    rospy.sleep(0.5)

    #print "This test/tutorial executes simple motions"\
    #    " in Cartesian impedance mode.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(2)
    print "Planner init ok"

    
    print "loading octomap..."
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        
        for i in range(4):    
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                return
            else:
                print "The trajectory could not be completed, retrying..."
                continue
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            if not isConfigurationClose(q_dest, js[1]):
                exitError(6)
    print('executing ended')

    if velma.enableMotors() != 0:
        exitError(14)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(3)

    closeBothHands(velma)

    
    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        print "Moving to required starting pose"
        js_init = velma.getLastJointState()
        planAndExecute(q_map_starting)
        #exitError(10)

    #wybor reki podnaszacej w zaleznosci od tego gdzie poczatkowo znajduje sie obiekt do przeniesienia
    T_B_S = velma.getTf("B", "sloik")
    if T_B_S.p.y() < 0:
        print "object is on the right"
        hand = 'right'          
    else:
        print "Object is on the left"
        hand = 'left'

    # get initial configuration
    js_init = velma.getLastJointState()
    #move above tables
    planAndExecute(q_map_above_tables)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)

    print "Reset tools for both arms..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_B_Wl = velma.getTf("B", "Wl")
    if not velma.moveCartImpRight([T_B_Wr], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if not velma.moveCartImpLeft([T_B_Wl], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)

    openHand(velma, hand)

    print "Moving " +hand+ " hand to pregrasp_2"
    moveHandToPosCart(velma, "pregrasp_2", hand)

    print "Moving " +hand+ " hand to pregrasp_1"
    moveHandToPosCart(velma, "pregrasp_1", hand)

    init_deg = 54   #poczatkowa liczba stopni okreslajaca zamkniecie dloni (wartosc dobrana na podstawie eksperymentow)
    graspJar(velma, init_deg, hand)
        
    print "Moving " +hand+ " hand to pregrasp_2"
    moveHandToPosCart(velma, "pregrasp_2", hand)

    if hand == 'left':
        planAndExecute(q_map_before_drop_left) 
    elif hand =='right':
        planAndExecute(q_map_before_drop_right) 

    #print to see whats wrong - potem do usuniecia
    js = velma.getLastJointState()
    print(js[1])

    #print "Moving " +hand+ " hand to predrop_1"
    #moveHandToPosCart(velma, "predrop_1", hand)

    openHand(velma, hand)
    
    print "Moving " +hand+ " hand to predrop_1"
    moveHandToPosCart(velma, "afterdrop_1", hand)

    closeBothHands(velma)

    planAndExecute(q_map_starting)
    openBothHands(velma)
    
    exitError(0)

