#!/usr/bin/env python
import rospy
import actionlib
import simulation_control.msg
from std_msgs.msg import Bool, String
from flying_controller import Flying

from geometry_msgs.msg import PoseStamped, Point


class goto_position_server():
    def __init__(self):

        # variables
        self.target_reached = False
        self.detected = False
        self.fly_mode = "regular"
        self.object_pose = Point()
        self.local_pose = PoseStamped()

        # publishers
        self.pose_control = rospy.Publisher('/position_control/set_position', PoseStamped, queue_size=10)
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)

        # subscribers
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/position_control/set_fly_mode', String, self.set_mode)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.goto_positionResult()
        self.action_server = actionlib.SimpleActionServer('goto_position', simulation_control.msg.goto_positionAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

    def execute_cb(self, goal):
        self.mode_control.publish('posctr')
        self.pose_control.publish(goal.destination)
        rospy.sleep(0.1)
        if self.fly_mode == "checkdrone":
            print("CheckForDrone")
            while not self.target_reached and self.detected:
                self.rate.sleep()
            if not self.target_reached and not self.detected:
                rospy.loginfo("Drone dropped" + str(self.local_pose))
                self.pose_control.publish(self.local_pose)
                self.action_server.set_aborted()
            else:
                rospy.loginfo("Destination reached-checkdrone")
                self.action_server.set_succeeded()
        elif self.fly_mode == "regular":
            print("Dont CheckForDrone")
            while not self.target_reached:
                self.rate.sleep()
            rospy.loginfo("Destination reached")
            self.action_server.set_succeeded()
        else:
            print("no such fly_mode: " + self.fly_mode)

    def distance_reached_cb(self, data):
        self.target_reached = data.data

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data

    def set_mode(self, data):
        print("Fly mode changed: " + str(data.data))
        self.fly_mode = data.data


if __name__ == '__main__':
    try:

        rospy.init_node('goto_position_server')
        goto_position_server()
    except rospy.ROSInterruptException:
        pass
