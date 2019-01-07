#!/usr/bin/env python

import roslib
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped

from long_grippers import LongGrippers
from short_grippers import ShortGrippers
from flying_controller import Flying
from object_detection import ObjectDetection


import time
from simulation_control.msg import descend_on_objectAction, descend_on_objectGoal, detect_objectAction, \
    detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, \
    long_grippersAction, long_grippersGoal, center_on_objectAction, center_on_objectGoal
from std_msgs.msg import Float32, String
from State import State
from StateMachine import StateMachine

# found below "import roslib"
roslib.load_manifest('simulation_control')

# Possible inputs: Object.Lost, Object.Found, Destination.Reached

# Start
class Start(State):
    takeoff = False
    local_pose = PoseStamped()

    def do_action(self):
        # CPSVO2018.goto_position_goal.destination.pose.position.z = 5
        # CPSVO2018.goto_position_client.send_goal(CPSVO2018.goto_position_goal)
        # Will return True or False
        # self.takeoff = CPSVO2018.goto_position_client.wait_for_result()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        self.takeoff = Flying.ascend_to_z(5, self.local_pose)

        # Move long grippers to pickup position
        LongGrippers.move_to_pickup_pos()

    def next_state(self):
        # if state_input == "":
        # return "Fly To Search pos"
        if self.takeoff:
            return Drone.FlyToS
        else:
            return Drone.Start

    def _local_pose_callback(self, data):
        self.local_pose = data


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
        print("FlyToDrone!")
        # ObjectDetection.detect_object()
        position = ObjectDetection.get_detection_position()
        self.success = Flying.fly_to_position(position.x, position.y, position.z)

    def next_state(self):
        if self.success:
            return Drone.DescendOnDrone
        else:
            rospy.sleep(100)
            return Drone.FlyToDrone


class Search(State):
    found = False

    local_pose = PoseStamped()

    def do_action(self):
        print("Searching")
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        ObjectDetection.start_object_detection()
        self.result = ObjectDetection.detect_object()
        print("search-drone-pos: " + str(ObjectDetection.get_detection_position()))
        obj = ObjectDetection.get_detection_position()
        if obj.x is not 0 and obj.y is not 0 and obj.z is not 0:
            self.found = True
        print("-------------")

    def next_state(self):
        if self.found:
            return Drone.FlyToDrone
        else:
            Flying.ascend_z_by(0.5, self.local_pose)
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


    def next_state(self):
        # if within x, return descend. If outside, return centering
        if self.detected is True:
            if self.local_pose.pose.position.z > 1.0:
                if abs(self.object_pose.x) > 0.3 or abs(self.object_pose.y) > 0.3:
                    return Drone.Centering
                else:  # abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                    return Drone.DescendOnDrone
            if 1.0 > self.local_pose.pose.position.z > 0.2:
                if abs(self.object_pose.x > 0.075 or abs(self.object_pose.y) > 0.075):
                    return Drone.Centering
                else:  # abs(self.object_pose.x) < 0.05 and abs(self.object_pose.y) < 0.05:
                    return Drone.DescendOnDrone
            if abs(self.object_pose.x) < 0.075 and abs(self.object_pose.y) < 0.075:
                return Drone.Land
            else:
                return Drone.Centering
        else:
            # Drone.Flying.pos = [-105, 0, 3]
            # return Drone.Flying
            print("Return Search after ascend by 0.5")
            Flying.ascend_z_by(0.5, self.local_pose)
            return Drone.Search

    def _local_pose_callback(self, data):
        self.local_pose = data

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


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
                if abs(self.object_pose.x) < 0.4 and abs(self.object_pose.y) < 0.4:
                    return Drone.DescendOnDrone
                else:
                    return Drone.Centering
            elif 1.0 > self.local_pose.pose.position.z > 0.2:
                if abs(self.object_pose.x) < 0.1 and abs(self.object_pose.y) < 0.1:
                    return Drone.DescendOnDrone
                else:
                    return Drone.Centering
            elif abs(self.object_pose.x) < 0.075 and abs(self.object_pose.y) < 0.075:
                print("Return Laaaand")
                return Drone.Land
            else:
                return Drone.Centering
        else:
            print("Return Search")
            # Flying.ascend_z_by(1, self.local_pose)
            return Drone.Search

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
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.sleep(2.0)
        # Will return True or False
        self.ascended = Flying.ascend_z_by(1, self.local_pose)  # MARK: aasdasd

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

    def _local_pose_callback(self, data):
        self.local_pose = data


class Land(State):

    detected = False
    object_pose = Point()
    local_pose = PoseStamped()
    landed = False

    def do_action(self):
        print("Land this bitch and check the camera pos")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        max_output = rospy.Publisher('/position_control/set_max_pid', Float32, queue_size=10)
        max_output_data = Float32()
        max_output_data.data = 0.2
        rospy.sleep(2.0)

        max_output.publish(max_output_data)
        # Remove sleep + if-statement if not working
        print("in land: pos" + str(self.local_pose))
        if self.local_pose.pose.position.z < 1.0:
            Flying.land_drone()
            self.landed = True
            rospy.sleep(2.0)
        # self.vel_control.publish(self.des_pose)
        # self.rate.sleep()
        # self.result.position_reached.data = True
        # self.action_server.set_succeeded(self.result)

    def next_state(self):
        # Return Close grippers if drone is visible. If not return Takeoff?
        if self.detected and self.landed:
            print("Drone is visible, close grippers")
            # Drone.ShortGrippers.gripperValue = 1.2200000000
            return Drone.Close_Short_Grippers
        # Drone.Flying.pos = [-105, 0, 3]
        else:
            print("Drone is not visible, take off!")
            if self.landed:
                Flying.arm_drone()
                rospy.sleep(3.0)

            Flying.ascend_to_z(1, self.local_pose)

            return Drone.FlyToDrone  # TAKEOFF FIRST??

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data


class Close_Short_Grippers(State):
    detected = False
    object_pose = Point()
    def do_action(self):
        print("CloseShortGrippers")

        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.sleep(2.0)
        print("obj pos: " + str(self.object_pose))
        rospy.sleep(3)
        ShortGrippers.start_grippers()
        ShortGrippers.close_grippers()
        Flying.arm_drone()
        rospy.sleep(2.5)

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
            self.object_pose = data
        else:
            self.detected = False


class Ascend_W_Drone(State):
    successAscend = False
    detected = False
    local_pose = PoseStamped()

    def do_action(self):
        print("Ascending to 0.5 m to close Long grippers")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.sleep(1.0)

        self.successAscend = Flying.ascend_to_z(0.5, self.local_pose)
        # rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        LongGrippers.close_grippers()  # Move this one
        Flying.CheckForDrone = True

    def next_state(self):
        if self.detected and self.successAscend:
            print("Drone detected")
            return Drone.Close_Long_Grippers
        else:
            print("Drone not detected, going to Ascend")
            LongGrippers.move_to_pickup_pos()
            ShortGrippers.open_grippers()
            Flying.CheckForDrone = False
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data


class Close_Long_Grippers(State):
    detected = False

    def do_action(self):
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        LongGrippers.close_grippers()
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
        print("Flying to drop position and changing fly mode...")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        self.set_mode = rospy.Publisher('/position_control/set_fly_mode', String, queue_size=10)
        rospy.sleep(1.0)
        self.set_mode.publish("checkdrone")
        self.success = Flying.fly_to_position(self.pos[0], self.pos[1], self.pos[2])
        # print("Did flying go well? : " + str(self.success))

    def next_state(self):
        if self.success and self.detected:
            print("Drone made it do destination and sees the object")
            self.set_mode.publish("regular")
            return Drone.Descend_W_Drone
        else:
            print("Dropped the object. Ascending to 3 m and searching, changing fly mode")
            self.set_mode.publish("regular")
            LongGrippers.move_to_pickup_pos()
            ShortGrippers.open_grippers()
            return Drone.Ascend

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
        else:
            self.detected = False


class Descend_W_Drone(State):
    detected = False
    local_pose = PoseStamped()

    def do_action(self):
        print("Descending")
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.sleep(1)
        Flying.ascend_to_z(1, self.local_pose)
        Flying.CheckForDrone = False

    def next_state(self):
        return Drone.Drop_D

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data


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
        return Drone.End


class End(State):

    def do_action(self):
        print("Landing")
        Flying.fly_to_position(0, -105, 0.5)

        Flying.land_drone()

    def next_state(self):
        return Drone.Done


class Done(State):

    def do_action(self):
        print("Done, should only print once")
        rospy.signal_shutdown("Drone has completed")

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


    def __init__(self):
        # Initialize variables and start some servers
        rospy.init_node('action_controller')  # Create a Node
        rospy.loginfo('Setting offboard')

        Flying.start_position_server()  # Starting position server.


        print("Init long grippers")  # This will start the Longgrippersserver
        LongGrippers.start_grippers()
        print("Init done")


        # End of Initializing

        rospy.loginfo("Taking off")

        d = Drone()
        d.start()  # This will perform the TakeOff action.

        while not rospy.is_shutdown():
            d.do_next_state()


if __name__ == '__main__':
    cps_vo_2018 = CPSVO2018()
