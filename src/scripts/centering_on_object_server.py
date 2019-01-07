#!/usr/bin/env python
import rospy
import actionlib
import simulation_control.msg
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String, Float32


class center_on_object_server():
    def __init__(self):

        # variables
        self.local_pose = PoseStamped()
        self.des_pose = PoseStamped()
        self.object_pose = Point()

        # publishers
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)
        self.pose_control = rospy.Publisher('/position_control/set_position', PoseStamped, queue_size=10)
        self.vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)

        # subscribers
        rospy.Subscriber('/color_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.center_on_objectResult()
        self.action_server = actionlib.SimpleActionServer('center_on_object',
                                                          simulation_control.msg.center_on_objectAction,
                                                          execute_cb=self.execute_cb,
                                                          auto_start=False)
        self.last_object_pose = Point()
        print("Start center server")

        set_x_pid = rospy.Publisher('/position_control/set_x_pid', Point, queue_size=10)
        set_y_pid = rospy.Publisher('/position_control/set_y_pid', Point, queue_size=10)
        set_z_pid = rospy.Publisher('/position_control/set_z_pid', Point, queue_size=10)

        # Used for reference, remove when done!
        # self.vController.set_x_pid(2.8, 0.913921, 0.0, self.max_output)
        # self.vController.set_y_pid(2.8, 0.913921, 0.0, self.max_output)  # 2.1, 0.713921, 0.350178
        # self.vController.set_z_pid(1.3, 2.4893, 0.102084, 0.1)

        # rospy.sleep(2.0)

        XnY = Point()
        Z = Point()

        XnY.x = 2.8
        XnY.y = 0.95
        XnY.z = 0.0

        Z.x = 1.1
        Z.y = 2.6
        Z.z = 0.122084

        # set_x_pid.publish(XnY)
        # set_y_pid.publish(XnY)


        rospy.sleep(1.0)

        self.action_server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Waiting to stabilize...")
        # self.mode_control.publish('velctr')
        rospy.sleep(5)
        rospy.loginfo("Now centering...")

    # while self.local_pose.pose.position.z > 1.0:
        self.rate.sleep()
        print("x = ", self.local_pose.pose.position.x)
        print("y = ", self.local_pose.pose.position.y)
        print("z = ", self.local_pose.pose.position.z)

        self.last_object_pose = self.object_pose
        self.des_pose.pose.position.x = self.local_pose.pose.position.x + self.object_pose.x
        self.des_pose.pose.position.y = self.local_pose.pose.position.y + self.object_pose.y
        self.des_pose.pose.position.z = self.local_pose.pose.position.z
        self.pose_control.publish(self.des_pose)
        # self.vel_control.publish(self.des_pose)
        rospy.loginfo("Only-Centering..." + str(self.object_pose))
        while not self.target_reached:
            rospy.sleep(1)

        print("Centering Done!")
        self.vel_control.publish(self.des_pose)
        self.rate.sleep()
        self.result.centered.data = True
        self.action_server.set_succeeded(self.result)

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def _local_pose_callback(self, data):
        self.local_pose = data

    def distance_reached_cb(self, data):
        # print("Distance reached!!" + str(data.data))
        self.target_reached = data.data


if __name__ == '__main__':
    try:

        rospy.init_node('center_on_object_server')
        center_on_object_server()
    except rospy.ROSInterruptException:
        pass
