#!/usr/bin/env python
import rospy
import actionlib
import simulation_control.msg

from geometry_msgs.msg import PoseStamped, Point
class detect_object_server():
    def __init__(self):

        #variables
        self.local_pose = PoseStamped()
        self.detected = False

        #publishers

        #subscribers
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.detect_objectResult()
        self.action_server = actionlib.SimpleActionServer('detect_object', simulation_control.msg.detect_objectAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def execute_cb(self, goal):
        rospy.loginfo("Waiting to stabilize...")
        rospy.sleep(5.0)
        rospy.loginfo("Now searching...")

        now = rospy.get_rostime()
        last_req = rospy.get_rostime()
        while not self.detected and (last_req - now) < rospy.Duration(5):
            self.rate.sleep()
            last_req = rospy.get_rostime()

        if self.detected:
            rospy.loginfo("Target Detected")
            self.result.detected_position = self.local_pose
            self.action_server.set_succeeded(self.result)
        else:
            rospy.loginfo("Target not Detected")
            self.action_server.set_aborted()




    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False


    def _local_pose_callback(self, data):
        self.local_pose = data

if __name__ == '__main__':
    try:

        rospy.init_node('detect_object_server')
        detect_object_server()
    except rospy.ROSInterruptException:
        pass
