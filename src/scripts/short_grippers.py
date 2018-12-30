import rospy
import actionlib
from simulation_control.msg import short_grippersGoal, short_grippersAction
from std_msgs.msg import Float32


class ShortGrippers:
    short_grippers_client = None

    @classmethod
    def start_grippers(cls):
        print("init short grippers...")
        cls.short_grippers_client = actionlib.SimpleActionClient('short_grippers', short_grippersAction)
        cls.short_grippers_client.wait_for_server()
        print("init done!")

    @classmethod
    def move_to_pos(cls, pos):
        rospy.loginfo("Moving short grippers to: " + str(pos))
        long_grippers_goal = short_grippersGoal(grip_rad_goal=Float32(pos))
        cls.short_grippers_client.send_goal(long_grippers_goal)
        cls.short_grippers_client.wait_for_result()
        if cls.short_grippers_client.get_result().goal_reached.data:
            rospy.loginfo("Short grippers moved to position")
        else:
            rospy.loginfo("Error with moving short grippers to position")

    @classmethod
    def open_grippers(cls):
        print("Open grippers...")
        goalposition = short_grippersGoal(grip_rad_goal=Float32(0.00000000000))
        cls.short_grippers_client.send_goal(goalposition)
        s = cls.short_grippers_client.wait_for_result()
        if s:
            print("Grippers Opened")
            return s
        else:
            print("Grippers didn't open.")
            return s

    @classmethod
    def close_grippers(cls):
        print("Closing grippers...")
        goalposition = short_grippersGoal(grip_rad_goal=Float32(1.57000000000))
        cls.short_grippers_client.send_goal(goalposition)
        s = cls.short_grippers_client.wait_for_result()
        if s:
            print("Grippers Closed")
            return s
        else:
            print("Grippers didn't close.")
            return s

    @classmethod
    def move_to_pickup_pos(cls):
        print("Move Grippers to PickupPos")
        goalposition = short_grippersGoal(grip_rad_goal=Float32(0.50000000000))
        cls.short_grippers_client.send_goal(goalposition)
        s = cls.short_grippers_client.wait_for_result()
        if s:
            print("Grippers moved to pickup pos")
            return s
        else:
            print("Grippers didn't move to pickup.")
            return s
