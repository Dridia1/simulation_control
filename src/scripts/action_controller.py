#!/usr/bin/env python

import roslib
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped

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
    CheckDroneVisible = False
    pos = [0, 0, 0]
    Done = False
    def do_action(self):
        if self.Done:
            mv_state.land(0.0)
        print("Fstate: Flying to " + str(self.pos[0]) + ", " + str(self.pos[1]) + ", " + str(self.pos[2]))
        fly_to_pos(self.pos[0], self.pos[1], self.pos[2])

    def next_state(self):
        if self.CheckDroneVisible:
            if detected:
                return Drone.Flying
            else:
                return Drone.Searching
        if self.Done:
            self.Done = False
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
            return Drone.Centering
        else:
            return Drone.Searching

class descend_on_object(State):
    local_pose = PoseStamped()
    des_pose = PoseStamped()
    object_pose = Point()
    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)

    def do_action(self):
        while self.local_pose.pose.position.z > 1.0:
            self.rate.sleep()
            print("x = ",self.local_pose.pose.position.x)
            print("y = ",self.local_pose.pose.position.y)
            print("z = ",self.local_pose.pose.position.z)
            rospy.sleep(0.2)
            if detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                self.last_object_pose = self.object_pose
                self.des_pose.pose.position.x = self.object_pose.x
                self.des_pose.pose.position.y = self.object_pose.y
                self.des_pose.pose.position.z = self.local_pose.pose.position.z
                self.vel_control.publish(self.des_pose)
                rospy.loginfo("Centering...")
                while not self.target_reached:
                    rospy.sleep(2)

            elif detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                self.des_pose.pose.position.x = 0
                self.des_pose.pose.position.y = 0
                self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.9
                rospy.loginfo("Descending...")
                self.vel_control.publish(self.des_pose)
                while not self.target_reached:
                    rospy.sleep(2)

        while self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
            self.rate.sleep()
            print("x = ", self.local_pose.pose.position.x)
            print("y = ", self.local_pose.pose.position.y)
            print("z = ", self.local_pose.pose.position.z)
            rospy.sleep(0.2)

            if detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
                self.last_object_pose = self.object_pose
                self.des_pose.pose.position.x = self.object_pose.x
                self.des_pose.pose.position.y = self.object_pose.y
                self.vel_control.publish(self.des_pose)
                rospy.loginfo("Centering...")
                while not self.target_reached:
                    rospy.sleep(2)

            elif detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                self.des_pose.pose.position.x = 0
                self.des_pose.pose.position.y = 0
                self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.1
                rospy.loginfo("Descending...")
                self.vel_control.publish(self.des_pose)
                while not self.target_reached:
                    rospy.sleep(2)


class Centering(State):
    local_pose = PoseStamped()
    des_pose = PoseStamped()
    object_pose = Point()
    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
    def do_action(self):
        print("Centering")
        if self.local_pose.pose.position.z > 1.0:
            self.last_object_pose = self.object_pose
            self.des_pose.pose.position.x = self.object_pose.x
            self.des_pose.pose.position.y = self.object_pose.y
            self.des_pose.pose.position.z = self.local_pose.pose.position.z
            self.vel_control.publish(self.des_pose)
            rospy.loginfo("Centering...")
            while not self.target_reached:
                rospy.sleep(2)
        if self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
            self.last_object_pose = self.object_pose
            self.des_pose.pose.position.x = self.object_pose.x
            self.des_pose.pose.position.y = self.object_pose.y
            self.vel_control.publish(self.des_pose)
            rospy.loginfo("Centering...")
            while not self.target_reached:
                rospy.sleep(2)


    def next_state(self):
        # if within x, return descend. If outside, return centering
        if detected:
            if self.local_pose.pose.position.z > 1.0:
                if detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                    return Drone.Centering
                elif detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                    return Drone.DescendOnDrone
            if self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
                if detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
                    return Drone.Centering
                elif detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
            return Drone.Land
        else:
            Drone.Flying.pos = [-105, 0, 3]
            return Drone.Flying


class DescendOnDrone(State):
    local_pose = PoseStamped()
    des_pose = PoseStamped()
    object_pose = Point()
    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
    z = 0.5
    succeeded = False

    def do_action(self):
        print("Descending")
        if self.local_pose.pose.position.z > 1.0:
            self.des_pose.pose.position.x = 0
            self.des_pose.pose.position.y = 0
            self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.9
            rospy.loginfo("Descending...")
            self.vel_control.publish(self.des_pose)
            while not self.target_reached:
                rospy.sleep(2)

        if self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
            self.des_pose.pose.position.x = 0
            self.des_pose.pose.position.y = 0
            self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.1
            rospy.loginfo("Descending...")
            self.vel_control.publish(self.des_pose)
            while not self.target_reached:
                rospy.sleep(2)

        # descendOnObjectGoal = descend_on_objectGoal()
        # #descend_on_objectGoal.height = 2.0
        # descend_on_object_client.send_goal(descendOnObjectGoal)
        # succeeded = descend_on_object_client.wait_for_result()  # Returns if landing worked.

    def next_state(self):
        if detected:
            if self.local_pose.pose.position.z > 1.0:
                if detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                    return Drone.Centering
                elif detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                    return Drone.DescendOnDrone
            if self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
                if detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
                    return Drone.Centering
                elif detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
            return Drone.Land
        else:
            Drone.Flying.pos = [-105, 0, 3]
            return Drone.Flying # TAKEOFF?
        # #if below height z, return Open grippers. above height y descend.
        # if self.succeeded:
        #     return Drone.CloseGrippers()  # Landed on drone
        # return Drone.TakeOff

# Regular Descend is fly_to_pos with lower z value.
class Descend(State):

    def do_action(self):
        print("Descending")
        goto_position_goal.destination.pose.position.z = 0.5
        goto_position_client.send_goal(goto_position_goal)
        # Will return True or False
        self.ascended = goto_position_client.wait_for_result()


    def next_state(self):
        # if within x and below height y, return land. above height y descend. If outside, return centering
        if dropOff.dropoff:
            LongGrippers.gripperValue = 0.000000000
            return Drone.LongGrippers
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


class Land(State):
    def do_action(self):
        print("Landing")
        self.vel_control.publish(self.des_pose)
        self.rate.sleep()
        self.result.position_reached.data = True
        self.action_server.set_succeeded(self.result)

    def next_state(self):
        # Return Close grippers if drone is visible. If not return Takeoff?
        if detected:
            Drone.ShortGrippers.gripperValue = 1.2200000000
            return ShortGrippers
        Drone.Flying.pos = [-105, 0, 3]
        return Drone.Flying # TAKEOFF FIRST??


class ShortGrippers(State):
    gripperValue = 0

    def do_action(self):
        print("CloseShortGrippers")
        move_short_grippers(self.gripperValue)

    def next_state(self):
        # Close grippers
        if not dropOff.dropoff:
            LongGrippers.gripperValue = 1.5700000000
            return Drone.LongGrippers
        else:
            ShortGrippers.gripperValue = 0.000000000
            dropOff.dropoff = False
            Drone.Flying.CheckDroneVisible = False
            Drone.Flying.pos = [0, -105, 5]
            Flying.Done = True
            return Drone.Flying

class LongGrippers(State):
    gripperValue = 0
    def do_action(self):
        if not dropOff.dropoff:
            time.sleep(3)
            print('Setting offboard')
            mv_state.set_mode('OFFBOARD')
            print('Arming vehicle')
            mv_state.arm(True)
            time.sleep(1)
            rospy.loginfo("Lifting")
            fly_to_pos(-105, 0, 1.5)
            time.sleep(1)
        if detected:
            move_long_grippers(self.gripperValue)
            time.sleep(1)

    def next_state(self):
        if dropOff.dropoff:
            ShortGrippers.gripperValue = 0.0000000000
        Drone.Flying.CheckDroneVisible = True
        Drone.Flying.pos = [-105, 0, 3]
        return Drone.Flying

class dropOff(State):
    dropoff = True
    def do_action(self):
        print("..")

    def next_state(self):
        if detected:
            return Drone.Descend
        Drone.Flying.pos = [-105, 0, 3]
        return Drone.Flying # ?????????????????????????????????????????

class Drone(StateMachine):
    def __init__(self):
        # Starting state
        StateMachine.__init__(self, Drone.TakeOff)


# Initialize the states for the StateMachine
Drone.TakeOff = TakeOff()
Drone.Searching = Searching()
Drone.Flying = Flying()
Drone.Centering = Centering()
Drone.ShortGrippers = ShortGrippers()
Drone.LongGrippers = LongGrippers()
Drone.Ascend = Ascend()
Drone.Descend = Descend()
Drone.DescendOnDrone = DescendOnDrone()
Drone.Land = Land()
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

detected = False

def get_cam_pos_callback(data):
    if data.x != float("inf"):
        detected = True
        object_pose = data
    else:
        detected = False


if __name__ == '__main__':
    # Initialize variables and Take off
    rospy.Subscriber('/color_detection/cam_point', Point, get_cam_pos_callback)
    rospy.init_node('action_controller')
    rospy.loginfo('Setting offboard')
    mv_state = mavros_state.mavros_state()
    mv_state.set_mode('OFFBOARD')
    rospy.loginfo('Arming vehicle')
    mv_state.arm(True)
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()

    # Init Short grippers
    short_grippers_client = actionlib.SimpleActionClient('short_grippers', short_grippersAction)
    short_grippers_client.wait_for_server()

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
    while not drone_done:
        d.do_next_state()

    '''
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
    descend_on_objectGoal = 2.0
    descend_on_object_client.send_goal(descend_on_object_goal)
    descend_on_object_client.wait_for_result()
    if descend_on_object_client.get_result().position_reached.data:
        print("landing")
        mv_state.arm(False)
    else:
        rospy.loginfo("Couldnt land, exiting")
    time.sleep(3)

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
    '''
    # Commit landing for Drone
    mv_state.land(0.0)
