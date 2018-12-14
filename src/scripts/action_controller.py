#!/usr/bin/env python

import roslib
import rospy
import actionlib
import mavros_state
import time
from simulation_control.msg import descend_on_objectAction, descend_on_objectGoal, detect_objectAction, \
    detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, \
    long_grippersAction, long_grippersGoal
from std_msgs.msg import Float32
from State import State
from StateMachine import StateMachine

# found below "import roslib"
roslib.load_manifest('simulation_control')


# Possible inputs: Object.Lost, Object.Found, Destination.Reached


class TakeOff(State):
    takeoff = False

    def do_action(self):
        # print("TOstate: Start")
        goto_position_goal.destination.pose.position.z = 5
        goto_position_client.send_goal(goto_position_goal)
        # Will return True or False
        self.takeoff = goto_position_client.wait_for_result()
        # print("TOState succ: " + str(self.takeoff))
        # Close long grippers to pickup position
        move_long_grippers(0.5000000000)
        print("TOState: moved long gripp. Done")

    def next_state(self):
        # if state_input == "":
        # return "Fly To Search pos"
        if self.takeoff:
            Drone.Flying.pos = [-105, 0, 3]
            return Drone.Flying
        else:
            return Drone.TakeOff


class Flying(State):
    pos = [0, 0, 0]
    CheckDroneVisible = False
    def do_action(self):
        print("Fstate: Flying to " + str(self.pos[0]) + ", " + str(self.pos[1]) + ", " + str(self.pos[2]))
        fly_to_pos(self.pos[0], self.pos[1], self.pos[2])

    def next_state(self):
        return Drone.Searching


class Searching(State):
    found = False

    def do_action(self):
        print("Searching")
        detect_object_client.send_goal(detect_object_goal)
        detect_object_client.wait_for_result()
        self.found = detect_object_client.get_result()

    def next_state(self):
        if self.found:
            Flying.pos = [detect_object_client.get_result().detected_position.pose.position.x,
                          detect_object_client.get_result().detected_position.pose.position.y,
                          detect_object_client.get_result().detected_position.pose.position.z]
            return Drone.Flying
        else:
            return Searching


class Centering(State):
    def do_action(self):
        print("Centering")

    def next_state(self):
        # if within x, return descend. If outside, return centering
        return Drone.TakeOff


# Regular Descend is fly_to_pos with lower z value.
class Descend(State):

    def do_action(self):
        print("Descending")
        goto_position_goal.destination.pose.position.z = 5
        goto_position_client.send_goal(goto_position_goal)
        # Will return True or False
        self.ascended = goto_position_client.wait_for_result()

    def next_state(self):
        # if within x and below height y, return land. above height y descend. If outside, return centering
        return Drone.TakeOff


class Ascend(State):
    def do_action(self):
        print("Ascending")
        goto_position_goal.destination.pose.position.z = 5
        goto_position_client.send_goal(goto_position_goal)
        # Will return True or False
        self.ascended = goto_position_client.wait_for_result()

    def next_state(self):
        if self.ascended:
            # If Camera sees drone Return Centering else return Ascend
            return Drone.Ascend
        else:
            return Drone.Ascend


class DescendOnDrone(State):
    z = 0.5
    succeeded = False

    def do_action(self):
        print("Descending")
        descendOnObjectGoal = descend_on_objectGoal()
        # descend_on_objectGoal.height = 2.0
        descend_on_object_client.send_goal(descendOnObjectGoal)
        succeeded = descend_on_object_client.wait_for_result()  # Returns if landing worked.

    def next_state(self):
        # if below height z, return Open grippers. above height y descend.
        if self.succeeded:
            return Drone.CloseGrippers()  # Landed on drone
        return Drone.TakeOff


class Land(State):
    def do_action(self):
        print("Land")

    def next_state(self):
        # Return Close grippers if drone is visible. If not return Takeoff?
        return Drone.TakeOff


class CloseGrippers(State):
    def do_action(self):
        print("CloseShortGrippers")

    def next_state(self):
        # Close grippers
        return Drone.TakeOff


class Drone(StateMachine):
    def __init__(self):
        # Starting state
        StateMachine.__init__(self, Drone.TakeOff)


# Initialize the states for the StateMachine
Drone.TakeOff = TakeOff()
Drone.Searching = Searching()
Drone.Flying = Flying()
Drone.Centering = Centering()
Drone.CloseGrippers = CloseGrippers()
Drone.Ascend = Ascend()
drone_done = False


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
    rospy.init_node('action_controller')
    rospy.loginfo('Setting offboard')
    mv_state = mavros_state.mavros_state()
    mv_state.set_mode('OFFBOARD')
    rospy.loginfo('Arming vehicle')
    mv_state.arm(True)
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()

    # Init Long Grippers
    long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
    long_grippers_client.wait_for_server()

    # Init Detect Object
    detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
    detect_object_client.wait_for_server()
    detect_object_goal = detect_objectGoal()

    # Init Descend on Object server
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()

    # End of Initializing

    rospy.loginfo("Taking off")
    # goto_position_goal.destination.pose.position.z = 5
    # goto_position_client.send_goal(goto_position_goal)
    # Will return True or False
    # s = goto_position_client.wait_for_result()
    d = Drone()
    d.start()  # This will perform the TakeOff action.
    # while not drone_done:
    # d.do_next_state()

    rospy.loginfo("Takeoff succeeded: ")

    fly_to_pos(-105, 0, 3)
    rospy.loginfo("Arrived at search position.")

    rospy.loginfo("Searching...")
    detect_object_client.send_goal(detect_object_goal)
    detect_object_client.wait_for_result()
    print(detect_object_client.get_result())

    rospy.loginfo("Going to detected position")
    fly_to_pos(detect_object_client.get_result().detected_position.pose.position.x,
               detect_object_client.get_result().detected_position.pose.position.y,
               detect_object_client.get_result().detected_position.pose.position.z)
    rospy.loginfo("Is at position (x,y,z)=({}, {}, {})".format(
        detect_object_client.get_result().detected_position.pose.position.x,
        detect_object_client.get_result().detected_position.pose.position.y,
        detect_object_client.get_result().detected_position.pose.position.z))

    rospy.loginfo("Descending on object")
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()
    rospy.loginfo("Descending server started")
    descend_on_object_goal = descend_on_objectGoal()
    # descend_on_objectGoal = 2.0
    descend_on_object_client.send_goal(descend_on_object_goal)
    descend_on_object_client.wait_for_result()
    if descend_on_object_client.get_result().position_reached.data:
        print("landing")
        mv_state.arm(False)
    else:
        rospy.loginfo("Couldnt land, exiting")
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

    rospy.loginfo("Descending prior to drop off")
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
