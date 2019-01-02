import roslib
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from simulation_control.msg import long_grippersGoal, long_grippersAction
from std_msgs.msg import Float32

import mavros_state
import time


class LongGrippers:
    long_grippers_client = None

    @classmethod
    def start_grippers(cls):
        print("init grippers...")
        cls.long_grippers_client = actionlib.SimpleActionClient('long_grippers', long_grippersAction)
        cls.long_grippers_client.wait_for_server()
        print("init done!")

    @classmethod
    def move_to_pos(cls, pos):
        rospy.loginfo("Moving long grippers to: " + str(pos))
        long_grippers_goal = long_grippersGoal(grip_rad_goal=Float32(pos))
        cls.long_grippers_client.send_goal(long_grippers_goal)
        cls.long_grippers_client.wait_for_result()
        if cls.long_grippers_client.get_result().goal_reached.data:
            rospy.loginfo("Long grippers moved to position")
        else:
            rospy.loginfo("Error with moving long grippers to position")

    @classmethod
    def open_grippers(cls):
        print("Open grippers...")
        goalposition = long_grippersGoal(grip_rad_goal=Float32(0.00000000000))
        cls.long_grippers_client.send_goal(goalposition)
        s = cls.long_grippers_client.wait_for_result()
        if s:
            print("Grippers Closed")
            return s
        else:
            print("Grippers didn't close.")
            return s

    @classmethod
    def close_grippers(cls):
        print("Closing grippers...")
        goalposition = long_grippersGoal(grip_rad_goal=Float32(1.57000000000))
        cls.long_grippers_client.send_goal(goalposition)
        s = cls.long_grippers_client.wait_for_result()
        if s:
            print("Grippers Closed all the way")
            return s
        else:
            print("Grippers didn't close.")
            return s

    @classmethod
    def move_to_pickup_pos(cls):
        print("Move Grippers to PickupPos")
        goalposition = long_grippersGoal(grip_rad_goal=Float32(0.50000000000))
        cls.long_grippers_client.send_goal(goalposition)
        s = cls.long_grippers_client.wait_for_result()
        if s:
            print("Grippers moved to pickup pos")
            return s
        else:
            print("Grippers didn't move to pickup.")
            return s
