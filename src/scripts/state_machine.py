#!/usr/bin/env python

# import roslib
# roslib.load_manifest('simulation_control')
# import rospy
# import actionlib
# import time

# from simulation_control.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal

# from std_msgs.msg import Float32


class StateMachine():
    startupMode = True
    START_UP = "start_up"
    FLYING_WITHOUT_DRONE = "flying_without_drone"
    SEARCHING = "searching"
    FLY_TO_SEARCH = "fly_to_search"

    def get_state(self):
        if self.startupMode:
            return self.START_UP
        else:
            return self.SEARCHING




