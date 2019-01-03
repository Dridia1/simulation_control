import rospy
import actionlib
from simulation_control.msg import goto_positionAction, goto_positionGoal
from std_msgs.msg import Float32

import mavros_state


class Flying:
    goto_position_client = None
    CheckForDrone = False
    mv_state = None

    @classmethod
    def start_position_server(cls):
        cls.mv_state = mavros_state.mavros_state()
        # cls.mv_state = mavros_state.mavros_state()
        if cls.mv_state is None:
            print("!!!!!!!!!!!!!!!!!!!!!!MV_STATE DID NOT GET SET!!!!!!!!!!!!!!!!!!!!!!")
        else:
            print("MV_STATE got set!! :) ")
        cls.mv_state.set_mode('OFFBOARD')
        # rospy.loginfo('Arming vehicle')
        cls.mv_state.arm(True)

        cls.goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
        cls.goto_position_client.wait_for_server()

    @classmethod
    def fly_to_position(cls, x, y, z):
        goto_position_goal = goto_positionGoal()
        goto_position_goal.destination.pose.position.x = x
        goto_position_goal.destination.pose.position.y = y
        goto_position_goal.destination.pose.position.z = z
        cls.goto_position_client.send_goal(goto_position_goal)
        return cls.goto_position_client.wait_for_result()

    @classmethod
    def ascend_to_z(cls, z):
        goto_position_goal = goto_positionGoal()
        goto_position_goal.destination.pose.position.z = z
        cls.goto_position_client.send_goal(goto_position_goal)
        return cls.goto_position_client.wait_for_result()

    @classmethod
    def land_drone(cls):
        print("land")
        cls.mv_state.arm(False)



