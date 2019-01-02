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

        # publishers
        self.pose_control = rospy.Publisher('/position_control/set_position', PoseStamped, queue_size=10)
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)

        # subscribers
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)

        self.rate = rospy.Rate(20)
        # self.result = simulation_control.msg.goto_positionResult()
        self.action_server = actionlib.SimpleActionServer('goto_position', simulation_control.msg.goto_positionAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def execute_cb(self, goal):
        self.mode_control.publish('posctr')
        self.pose_control.publish(goal.destination)
        rospy.sleep(0.1)
        print(self.target_reached)
        if Flying.CheckForDrone:
            while not self.target_reached and self.detected:
                self.rate.sleep()
            if not self.target_reached and not self.detected:
                rospy.loginfo("Drone dropped")
                self.action_server.set_aborted()
            rospy.loginfo("Destination reached")
            self.action_server.set_succeeded()
        elif not Flying.CheckForDrone:
            while not self.target_reached:
                self.rate.sleep()
            rospy.loginfo("Destination reached")
            self.action_server.set_succeeded()



    def distance_reached_cb(self, data):
        self.target_reached = data.data

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


if __name__ == '__main__':
    try:

        rospy.init_node('goto_position_server')
        goto_position_server()
    except rospy.ROSInterruptException:
        pass
