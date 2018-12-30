#!/usr/bin/env python

import roslib
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped

from long_grippers import LongGrippers

import mavros_state
import time
from simulation_control.msg import descend_on_objectAction, descend_on_objectGoal, detect_objectAction, \
    detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, \
    long_grippersAction, long_grippersGoal, center_on_objectAction, center_on_objectGoal
from std_msgs.msg import Float32
from State import State
from StateMachine import StateMachine

# found below "import roslib"
roslib.load_manifest('simulation_control')

# Possible inputs: Object.Lost, Object.Found, Destination.Reached

# Start
class TakeOff(State):
    takeoff = False

    def do_action(self):
        # print("TOstate: Start")
        CPSVO2018.goto_position_goal.destination.pose.position.z = 5
        CPSVO2018.goto_position_client.send_goal(CPSVO2018.goto_position_goal)
        # Will return True or False
        self.takeoff = CPSVO2018.goto_position_client.wait_for_result()
        # print("TOState succ: " + str(self.takeoff))
        # Close long grippers to pickup position
        print("Takeoff done, Grippers next, should fail the fuck out of thems")
        LongGrippers.close_grippers()
        # cps_vo_2018.grippers_long.move_to_pickup_pos()
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
            cps_vo_2018.mv_state.land(0.0)
        print("Fstate: Flying to " + str(self.pos[0]) + ", " + str(self.pos[1]) + ", " + str(self.pos[2]))
        cps_vo_2018.fly_to_pos(self.pos[0], self.pos[1], self.pos[2])

    def next_state(self):
        if self.CheckDroneVisible:
            if cps_vo_2018.detected:
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
        cps_vo_2018.detect_object_client.send_goal(cps_vo_2018.detect_object_goal)
        cps_vo_2018.detect_object_client.wait_for_result()
        self.found = cps_vo_2018.detect_object_client.get_result()

    def next_state(self):
        if self.found:
            Flying.pos = [cps_vo_2018.detect_object_client.get_result().detected_position.pose.position.x,
                          cps_vo_2018.detect_object_client.get_result().detected_position.pose.position.y,
                          cps_vo_2018.detect_object_client.get_result().detected_position.pose.position.z]
            return Drone.Centering
        else:
            return Drone.Searching


class DescendOnObject(State):
    local_pose = PoseStamped()
    des_pose = PoseStamped()
    object_pose = Point()
    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)

    def do_action(self):
        descend_on_object_goal = descend_on_objectGoal()
        cps_vo_2018.descend_on_object_client.send_goal(descend_on_object_goal)
        descend_succeeded = cps_vo_2018.descend_on_object_client.wait_for_result()

    def next_state(self):
        if cps_vo_2018.detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
            return Drone.DescendOnObject
        elif cps_vo_2018.detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
            return Drone.DescendOnObject
        elif cps_vo_2018.detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
            return Drone.Centering
        elif cps_vo_2018.detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
            return Drone.Centering


class Centering(State):
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
        if cps_vo_2018.detected:
            if self.local_pose.pose.position.z > 1.0:
                if cps_vo_2018.detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                    return Drone.Centering
                elif cps_vo_2018.detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                    return Drone.DescendOnDrone
            if self.local_pose.pose.position.z < 1.0 and self.local_pose.pose.position.z > 0.1:
                if cps_vo_2018.detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
                    return Drone.Centering
                elif cps_vo_2018.detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
            return Drone.Land
        else:
            Drone.Flying.pos = [-105, 0, 3]
            return Drone.Flying


class DescendOnDrone(State):
    des_pose = PoseStamped()  # Destination the drone are heading to
    object_pose = Point()
    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
    z = 0.5
    succeeded = False

    def do_action(self):
        print("Descending")
        if cps_vo_2018.local_pose.pose.position.z > 1.0:
            self.des_pose.pose.position.x = 0
            self.des_pose.pose.position.y = 0
            self.des_pose.pose.position.z = cps_vo_2018.local_pose.pose.position.z - 0.9
            rospy.loginfo("Descending...")
            self.vel_control.publish(self.des_pose)
            while not self.target_reached:
                rospy.sleep(2)

        if 1.0 > cps_vo_2018.local_pose.pose.position.z > 0.1:
            self.des_pose.pose.position.x = 0
            self.des_pose.pose.position.y = 0
            self.des_pose.pose.position.z = cps_vo_2018.local_pose.pose.position.z - 0.1
            rospy.loginfo("Descending...")
            self.vel_control.publish(self.des_pose)
            while not self.target_reached:
                rospy.sleep(2)

        # descendOnObjectGoal = descend_on_objectGoal()
        # #descend_on_objectGoal.height = 2.0
        # descend_on_object_client.send_goal(descendOnObjectGoal)
        # succeeded = descend_on_object_client.wait_for_result()  # Returns if landing worked.

    def next_state(self):
        if cps_vo_2018.detected:
            if cps_vo_2018.local_pose.pose.position.z > 1.0:
                if cps_vo_2018.detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                    return Drone.Centering
                elif cps_vo_2018.detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                    return Drone.DescendOnDrone
            if 1.0 > cps_vo_2018.local_pose.pose.position.z > 0.1:
                if cps_vo_2018.detected and (abs(self.object_pose.x) > 0.05 or abs(self.object_pose.y) > 0.05):
                    return Drone.Centering
                elif cps_vo_2018.detected and abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
            return Drone.Land
        else:
            Drone.Flying.pos = [-105, 0, 3]
            return Drone.Flying  # TAKEOFF?
        # #if below height z, return Open grippers. above height y descend.
        # if self.succeeded:
        #     return Drone.CloseGrippers()  # Landed on drone
        # return Drone.TakeOff


# Regular Descend is fly_to_pos with lower z value.
class Descend(State):
    def do_action(self):
        print("Descending")
        cps_vo_2018.goto_position_goal.destination.pose.position.z = 0.5
        cps_vo_2018.goto_position_client.send_goal(cps_vo_2018.goto_position_goal)
        # Will return True or False
        self.ascended = cps_vo_2018.goto_position_client.wait_for_result()

    def next_state(self):
        # if within x and below height y, return land. above height y descend. If outside, return centering
        if dropOff.dropoff:
            LongGrippers.gripperValue = 0.000000000
            return Drone.LongGrippers
        return Drone.TakeOff


class Ascend(State):
    def do_action(self):
        print("Ascending")
        cps_vo_2018.goto_position_goal.destination.pose.position.z = 5
        cps_vo_2018.goto_position_client.send_goal(cps_vo_2018.goto_position_goal)
        # Will return True or False
        self.ascended = cps_vo_2018.goto_position_client.wait_for_result()

    def next_state(self):
        if self.ascended:
            # If Camera sees drone Return Centering else return Ascend
            return Drone.Ascend
        else:
            return Drone.Ascend


class Land(State):
    def do_action(self):
        print("Landing")
        # self.vel_control.publish(self.des_pose)
        # self.rate.sleep()
        # self.result.position_reached.data = True
        # self.action_server.set_succeeded(self.result)

    def next_state(self):
        # Return Close grippers if drone is visible. If not return Takeoff?
        if cps_vo_2018.detected:
            Drone.ShortGrippers.gripperValue = 1.2200000000
            return ShortGrippers
        Drone.Flying.pos = [-105, 0, 3]
        return Drone.Flying  # TAKEOFF FIRST??


class ShortGrippers(State):
    gripperValue = 0

    def do_action(self):
        print("CloseShortGrippers")
        cps_vo_2018.move_short_grippers(self.gripperValue)

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
            cps_vo_2018.mv_state.set_mode('OFFBOARD')
            print('Arming vehicle')
            cps_vo_2018.mv_state.arm(True)
            time.sleep(1)
            rospy.loginfo("Lifting")
            cps_vo_2018.fly_to_pos(-105, 0, 1.5)
            time.sleep(1)
        if cps_vo_2018.detected:
            cps_vo_2018.move_long_grippers(self.gripperValue)
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
        if cps_vo_2018.detected:
            return Drone.Descend
        Drone.Flying.pos = [-105, 0, 3]
        return Drone.Flying  # ?????????????????????????????????????????


class Drone(StateMachine):
    def __init__(self):
        # Starting state
        StateMachine.__init__(self, Drone.TakeOff)


# Initialize the states for the States
Drone.TakeOff = TakeOff()
Drone.Searching = Searching()
Drone.Flying = Flying()
Drone.Centering = Centering()
Drone.ShortGrippers = ShortGrippers()
Drone.LongGrippers = LongGrippers()
Drone.Ascend = Ascend()
Drone.Descend = Descend()
Drone.DescendOnObject = DescendOnObject()
Drone.DescendOnDrone = DescendOnDrone()  # Duplicate?
Drone.Land = Land()
drone_done = False


class CPSVO2018:
    detected = False  # If the dead drone is detected by the Camera
    mv_state = None

    local_pose = PoseStamped()  # Local position of the drone

    # Things to consider deleting
    goto_position_goal = goto_positionGoal()
    detect_object_goal = detect_objectGoal()
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)

    grippers_long = LongGrippers()

    def __init__(self):
        # Initialize variables and Take off
        rospy.init_node('action_controller')
        rospy.loginfo('Setting offboard')
        mv_state = mavros_state.mavros_state()
        if mv_state is None:
            print("!!!!!!!!!!!!!!!!!!!!!!MV_STATE DID NOT GET SET!!!!!!!!!!!!!!!!!!!!!!")
        else:
            print("MV_STATE got set!! :) ")
        mv_state.set_mode('OFFBOARD')
        rospy.loginfo('Arming vehicle')
        mv_state.arm(True)

        # get_cam_pos_callback called when cam_point received
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        # _local_pose_callback called when pose received
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        # self.goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
        self.goto_position_client.wait_for_server()
        # self.goto_position_goal = goto_positionGoal()

        # Init Short grippers
        self.short_grippers_client = actionlib.SimpleActionClient('short_grippers', short_grippersAction)
        self.short_grippers_client.wait_for_server()

        # Init Long Grippers
        # self.long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
        print("Init long grippers")
        LongGrippers.start_grippers()
        print("Init done")

        # print("Move long grippers to pickup pos")
        # self.grippers_long.move_to_pickup_pos()
        # print("Move is done")


        # Init Detect Object
        self.detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
        self.detect_object_client.wait_for_server()
        # self.detect_object_goal = detect_objectGoal()

        # Init Center on Object server
        # self.center_on_object_client = actionlib.SimpleActionClient('asd', center_on_objectAction)
        # self.center_on_object_client.wait_for_server()

        # Init Descend on Object server
        self.descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
        self.descend_on_object_client.wait_for_server()

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

    def move_short_grippers(self, Spos):
        rospy.loginfo("Moving short grippers to: " + str(Spos))
        short_grippers_goal = short_grippersGoal(grip_rad_goal=Float32(Spos))
        self.short_grippers_client.send_goal(short_grippers_goal)
        self.short_grippers_client.wait_for_result()
        if self.short_grippers_client.get_result().goal_reached.data:
            rospy.loginfo("Short grippers moved to position")
        else:
            rospy.loginfo("Error when moving short grippers")

    def fly_to_pos(self, x, y, z):
        self.goto_position_goal.destination.pose.position.x = x
        self.goto_position_goal.destination.pose.position.y = y
        self.goto_position_goal.destination.pose.position.z = z
        self.goto_position_client.send_goal(self.goto_position_goal)
        self.goto_position_client.wait_for_result()

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data



if __name__ == '__main__':
    cps_vo_2018 = CPSVO2018()
