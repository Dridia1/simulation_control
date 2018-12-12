#!/usr/bin/env python

import roslib
roslib.load_manifest('simulation_control')
import rospy
import actionlib
import mavros_state
import time

from state_machine import StateMachine

from simulation_control.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal

from std_msgs.msg import Float32

# mv_state = None
# goto_position_client = None
# goto_position_goal = None


def init_takeoff():
    print("asd")


def move_long_grippers(pos):
    rospy.loginfo("Moving long grippers to: " + str(pos))
    long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(pos))
    long_grippers_client.send_goal(long_grippers_goal)
    long_grippers_client.wait_for_result()
    if long_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Long grippers moved to position")
    else:
        rospy.loginfo("Error with moving long grippers to position")


def move_short_grippers(pos):
    rospy.loginfo("Moving short grippers to: " + str(pos))
    short_grippers_goal = short_grippersGoal(grip_rad_goal=Float32(pos))
    short_grippers_client.send_goal(short_grippers_goal)
    short_grippers_client.wait_for_result()
    if short_grippers_client.get_result().goal_reached.data:
        rospy.loginfo("Short grippers moved to position")
    else:
        rospy.loginfo("Error when moving short grippers")


def fly_to_pos(x, y, z):
    goto_position_goal.destination.pose.position.x = x
    goto_position_goal.destination.pose.position.y = y
    goto_position_goal.destination.pose.position.z = z
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()


if __name__ == '__main__':
    # Initialize variables and Take off
    state = StateMachine()
    rospy.init_node('action_controller')
    rospy.loginfo('Setting offboard')
    mv_state = mavros_state.mavros_state()
    mv_state.set_mode('OFFBOARD')
    rospy.loginfo('Arming vehicle')
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    # Will return True or False
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    # Will return True or False
    s = goto_position_client.wait_for_result()
    if s:
        state.startupMode = False
    rospy.loginfo("Takeoff succeded: " + str(StateMachine.startupMode))
    # End of Initializing

    if state.get_state() == state.FLY_TO_SEARCH:
        print("Fly to search position")
    elif state.get_state() == state.SEARCHING:
        print("Searching")

    # Init Long Grippers
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()

    # Close long grippers to pickup position
    move_long_grippers(0.5000000000)

    fly_to_pos(-105, 0, 3)
    rospy.loginfo("Arrived at search position.")

    rospy.loginfo("Searching...")
    detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
    detect_object_client.wait_for_server()
    detect_object_goal = detect_objectGoal()
    detect_object_client.send_goal(detect_object_goal)
    detect_object_client.wait_for_result()
    print(detect_object_client.get_result())

    rospy.loginfo("Going to detected position")
    fly_to_pos(detect_object_client.get_result().detected_position.pose.position.x,
               detect_object_client.get_result().detected_position.pose.position.y,
               detect_object_client.get_result().detected_position.pose.position.z)
    rospy.loginfo("Is at position (x,y,z)=({}, {}, {})".format(detect_object_client.get_result().detected_position.pose.position.x,
                                                               detect_object_client.get_result().detected_position.pose.position.y,
                                                               detect_object_client.get_result().detected_position.pose.position.z))

    rospy.loginfo("Descending on object")
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()
    rospy.loginfo("Descending server started")
    descend_on_object_goal = descend_on_objectGoal()
    descend_on_objectGoal = 2.0
    descend_on_object_client.send_goal(descend_on_object_goal)
    descend_on_object_client.wait_for_result()
    if descend_on_object_client.get_result().position_reached.data:
        print("landing")
        mv_state.arm(False)
    else:
        rospy.loginfo("Couldnt land exiting")
    time.sleep(3)

    # Init Short grippers
    short_grippers_client = actionlib.SimpleActionClient('short_grippers', short_grippersAction)
    short_grippers_client.wait_for_server()

    # Close Short Grippers
    move_short_grippers(1.22000000000)

    time.sleep(3)
    print('Setting offboard')
    mv_state.set_mode('OFFBOARD')
    print('Arming vehicle')
    mv_state.arm(True)
    time.sleep(1)

    rospy.loginfo("Lifting")
    fly_to_pos(-105, 0, 1.5)
    time.sleep(1)

    # Close Long Grippers
    move_long_grippers(1.57000000000)
    time.sleep(1)

    rospy.loginfo("Going to drop off position")
    fly_to_pos(0, 105, 5)
    time.sleep(3)

    rospy.loginfo("Deceding prior to drop off")
    fly_to_pos(0, 105, 0.5)

    # Open Long Grippers
    time.sleep(3)
    move_long_grippers(0.00000000000)

    # Open short Grippers
    time.sleep(1)
    move_short_grippers(0.00000000000)
    time.sleep(3)

    rospy.loginfo("Going Home")
    fly_to_pos(0, -105, 5)

    # Commit landing for Drone
    mv_state.land(0.0)
