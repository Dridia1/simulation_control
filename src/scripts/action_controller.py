#!/usr/bin/env python

import roslib
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped

from long_grippers import LongGrippers
from short_grippers import ShortGrippers
from flying_controller import Flying
from object_detection import ObjectDetection

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
class Start(State):
    takeoff = False

    def do_action(self):
        # CPSVO2018.goto_position_goal.destination.pose.position.z = 5
        # CPSVO2018.goto_position_client.send_goal(CPSVO2018.goto_position_goal)
        # Will return True or False
        # self.takeoff = CPSVO2018.goto_position_client.wait_for_result()

        self.takeoff = Flying.ascend_to_z(5)

        # Move long grippers to pickup position
        LongGrippers.move_to_pickup_pos()

    def next_state(self):
        # if state_input == "":
        # return "Fly To Search pos"
        if self.takeoff:
            return Drone.FlyToS
        else:
            return Drone.Start


class FlyToS(State):
    pos = [-105, 0, 3]
    success = False

    def do_action(self):
        self.success = Flying.fly_to_position(self.pos[0], self.pos[1], self.pos[2])

    def next_state(self):
        if self.success:
            return Drone.Search
        else:
            return Drone.FlyToS


class FlyToDrone(State):

    success = False

    def do_action(self):
        position = ObjectDetection.get_detection_position()
        self.success = Flying.fly_to_position(position.x, position.y, position.z)

    def next_state(self):
        if self.success:
            return Drone.Centering
        else:
            return Drone.FlyToDrone


class Search(State):
    found = False

    local_pose = PoseStamped()

    def do_action(self):
        print("Searching")
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        ObjectDetection.start_object_detection()
        self.found = ObjectDetection.detect_object()

    def next_state(self):
        if self.found:
            return Drone.FlyToDrone
        else:
            Flying.ascend_z_by(3, self.local_pose)
            return Drone.Search

    def _local_pose_callback(self, data):
        self.local_pose = data


class Centering(State):
    des_pose = PoseStamped()

    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)


    centering_on_object_client = None

    detected = False

    def __init__(self):
        self.local_pose = PoseStamped()
        self.object_pose = Point()

    def do_action(self):

        # subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)

        rospy.sleep(2.0)
        print("centering!")



        print("Centering state")
        if self.centering_on_object_client is None:
            self.centering_on_object_client = actionlib.SimpleActionClient('center_on_object', center_on_objectAction)
            self.centering_on_object_client.wait_for_server()
            rospy.loginfo("Centering server started")

        center_on_object_goal = center_on_objectGoal()
        center_on_objectGoall = 2.0
        self.centering_on_object_client.send_goal(center_on_object_goal)
        done = self.centering_on_object_client.wait_for_result()
        print("centerdone?: " + str(done))
        if self.centering_on_object_client.get_result().centered.data:
            print("Centering done")
            # mv_state.arm(False)
        else:
            print("Centering didn't succeed")

        '''if self.local_pose.pose.position.z > 1.0:
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
        '''

    def next_state(self):
        # if within x, return descend. If outside, return centering
        if self.detected is True:
            if self.local_pose.pose.position.z > 1.0:
                if abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2:
                    return Drone.Centering
                elif abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                    return Drone.DescendOnDrone
            if 1.0 > self.local_pose.pose.position.z > 0.1:
                if abs(self.object_pose.x > 0.05 or abs(self.object_pose.y) > 0.05):
                    return Drone.Centering
                elif abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
            return Drone.Land
        else:
            # Drone.Flying.pos = [-105, 0, 3]
            # return Drone.Flying
            print("Return Search after ascend by 2")
            Flying.ascend_z_by(2, self.local_pose)
            return Drone.Search

    def _local_pose_callback(self, data):
        self.local_pose = data

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


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


class DescendOnDrone(State):
    des_pose = PoseStamped()  # Destination the drone are heading to
    object_pose = Point()
    vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
    z = 0.5
    succeeded = False

    detected = False
    descend_on_object_client = None
    local_pose = PoseStamped()

    def do_action(self):
        print("Descending")

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)

        if self.descend_on_object_client is None:
            self.descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
            self.descend_on_object_client.wait_for_server()
            rospy.loginfo("Descending server started")

        descend_on_object_goal = descend_on_objectGoal()
        descend_on_objectGoall = 2.0
        self.descend_on_object_client.send_goal(descend_on_object_goal)
        self.descend_on_object_client.wait_for_result()
        if self.descend_on_object_client.get_result().position_reached.data:
            print("Descending done")
            # mv_state.arm(False)
        else:
            print("Descending didn't succeed")

    def next_state(self):
        if self.detected:
            if self.local_pose.pose.position.z > 1.0:
                if abs(self.object_pose.x) > 0.5 or abs(self.object_pose.y) > 0.5:
                    return Drone.Centering
                elif abs(self.object_pose.x) < 0.6 and abs(self.object_pose.y) < 0.6:
                    return Drone.DescendOnDrone
                else:
                    return Drone.Centering
            elif 1.0 > self.local_pose.pose.position.z > 0.05:
                if abs(self.object_pose.x > 0.05 or abs(self.object_pose.y) > 0.05):
                    return Drone.Centering
                elif abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
                else:
                    return Drone.Centering
            elif abs(self.object_pose.x) < 0.03 and abs(self.object_pose.y) < 0.03:
                print("Return Laaaand")
                return Drone.Land
            else:
                return Drone.Centering
        else:
            print("Return FlyToS")
            Flying.ascend_z_by(1, self.local_pose)
            return Drone.Centering

    def _local_pose_callback(self, data):
        self.local_pose = data

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


class Ascend(State):
    ascended = False
    detected = False
    def do_action(self):
        print("Ascending")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)

        # Will return True or False
        self.ascended = Flying.ascend_z_by(1)

    def next_state(self):
        if self.ascended and self.detected:
            # If Camera sees drone Return Centering else return Ascend
            return Drone.Search
        else:
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

class Land(State):

    detected = False
    object_pose = Point()

    def do_action(self):
        print("Land this bitch and check the camera pos")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        Flying.land_drone()
        rospy.sleep(2.0)
        # self.vel_control.publish(self.des_pose)
        # self.rate.sleep()
        # self.result.position_reached.data = True
        # self.action_server.set_succeeded(self.result)

    def next_state(self):
        # Return Close grippers if drone is visible. If not return Takeoff?
        if self.detected:
            print("Drone is visible, close grippers")
            # Drone.ShortGrippers.gripperValue = 1.2200000000
            return Drone.Close_Short_Grippers
        # Drone.Flying.pos = [-105, 0, 3]
        else:
            print("Drone is not visible, take off!")
            Flying.arm_drone()
            rospy.sleep(3.0)
            Flying.ascend_to_z(2)
            return Drone.Search  # TAKEOFF FIRST??

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


class Close_Short_Grippers(State):
    detected = False

    def do_action(self):
        print("CloseShortGrippers")

        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)

        ShortGrippers.start_grippers()
        ShortGrippers.close_grippers()
        Flying.arm_drone()
        rospy.sleep(1.0)

    def next_state(self):
        if self.detected:
            print("Drone visible, ascending")

            return Drone.Ascend_W_Drone
        else:
            print("Drone not visible, going to Ascend")
            ShortGrippers.open_grippers()
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
        else:
            self.detected = False


class Ascend_W_Drone(State):
    successAscend = False
    detected = False

    def do_action(self):
        print("Ascending to 0.5 m to close Long grippers")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        self.successAscend = Flying.ascend_to_z(0.5)
        # rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        LongGrippers.close_grippers()  # Move this one
        Flying.CheckForDrone = True

    def next_state(self):
        if self.detected and self.successAscend:
            print("Drone detected")
            return Drone.Close_Long_Grippers
        else:
            print("Drone not detected, going to Ascend")
            Flying.CheckForDrone = False
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            print("detected")
            self.detected = True
        else:
            print("not detected")
            self.detected = False


class Close_Long_Grippers(State):
    detected = False

    def do_action(self):
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        # LongGrippers.close_grippers()
        rospy.sleep(1)
        print("Long grippers closed")

    def next_state(self):
        if self.detected:
            return Drone.FlyToDrop
        else:
            Flying.CheckForDrone = False
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
        else:
            self.detected = False


class FlyToDrop(State):
    pos = [0, 105, 5]
    success = False
    detected = False

    def do_action(self):
        print("Flying to drop position")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        self.success = Flying.fly_to_position(self.pos[0], self.pos[1], self.pos[2])

    def next_state(self):
        if self.success and self.detected:
            print("Drone made it do destination and sees the drone")
            return Drone.Descend_W_Drone
        else:
            print("Dropped the drone. Ascending to 3 m and searching")
            Flying.CheckForDrone = False
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
        else:
            self.detected = False


class Descend_W_Drone(State):
    detected = False

    def do_action(self):
        print("Descending")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        Flying.ascend_to_z(0.5)
        Flying.CheckForDrone = False

    def next_state(self):
        if self.detected:
            return Drone.Drop_D

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


class Drop_D(State):
    longG = False
    shortG = False

    def do_action(self):
        print("Opening long and short grippers")

        self.longG = LongGrippers.open_grippers()
        time.sleep(3)
        self.shortG = ShortGrippers.open_grippers()

    def next_state(self):
        if self.shortG and self.longG:
            return Drone.FlyToEnd


class FlyToEnd(State):

    def do_action(self):
        print("Going Home")
        Flying.fly_to_position(0, -105, 5)

    def next_state(self):
        return Drone.end


class End(State):

    def do_action(self):
        print("Landing")
        self.mv_state.land(0.0)

    def next_state(self):
        return Drone.Done

class Done(State):

    def do_action(self):
        print("Done")

    def next_state(self):
        return Drone.Done


class Drone(StateMachine):
    def __init__(self):
        # Starting state
        StateMachine.__init__(self, Drone.Start)


# Initialize the states for the States
Drone.Start = Start()
Drone.FlyToS = FlyToS()
Drone.FlyToDrone = FlyToDrone()
Drone.Land = Land()
Drone.Search = Search()
Drone.Centering = Centering()
Drone.Close_Short_Grippers = Close_Short_Grippers()
Drone.Ascend_W_Drone = Ascend_W_Drone()
Drone.FlyToDrop = FlyToDrop()
Drone.Descend_W_Drone = Descend_W_Drone()
Drone.Drop_D = Drop_D()
Drone.FlyToEnd = FlyToEnd()
Drone.End = End()
Drone.Done = Done()

Drone.Close_Long_Grippers = Close_Long_Grippers()
Drone.Ascend = Ascend()
Drone.DescendOnObject = DescendOnObject()
Drone.DescendOnDrone = DescendOnDrone() # Duplicate?

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
        '''self.mv_state = mavros_state.mavros_state()
        if self.mv_state is None:
            print("!!!!!!!!!!!!!!!!!!!!!!MV_STATE DID NOT GET SET!!!!!!!!!!!!!!!!!!!!!!")
        else:
            print("MV_STATE got set!! :) ")
        self.mv_state.set_mode('OFFBOARD')
        rospy.loginfo('Arming vehicle')
        self.mv_state.arm(True)'''
        Flying.start_position_server()

        # get_cam_pos_callback called when cam_point received
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        # _local_pose_callback called when pose received
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        Flying.start_position_server()
        # self.goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
        # self.goto_position_client.wait_for_server()
        # self.goto_position_goal = goto_positionGoal()

        # Init Short grippers
        # self.short_grippers_client = actionlib.SimpleActionClient('short_grippers', short_grippersAction)
        # self.short_grippers_client.wait_for_server()

        # Init Long Grippers
        # self.long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
        print("Init long grippers")
        LongGrippers.start_grippers()
        print("Init done")

        # print("Move long grippers to pickup pos")
        # self.grippers_long.move_to_pickup_pos()
        # print("Move is done")


        # Init Detect Object
        # self.detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
        # self.detect_object_client.wait_for_server()
        # self.detect_object_goal = detect_objectGoal()

        # Init Center on Object server
        # self.center_on_object_client = actionlib.SimpleActionClient('asd', center_on_objectAction)
        # self.center_on_object_client.wait_for_server()

        # Init Descend on Object server
        # self.descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
        # self.descend_on_object_client.wait_for_server()

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
        self.mv_state.land(0.0)

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
        return self.goto_position_client.wait_for_result()

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
