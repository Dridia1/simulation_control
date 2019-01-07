import rospy
import actionlib
from simulation_control.msg import detect_objectAction, detect_objectGoal
from std_msgs.msg import Float32


class ObjectDetection:
    detect_object_client = None
    running = False

    @classmethod
    def start_object_detection(cls):
        print("starting object server")
        if not cls.running:
            cls.detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
            cls.detect_object_client.wait_for_server()
            cls.running = True


    @classmethod
    def get_detection_position(cls):
        print("return object position")
        result = cls.detect_object_client.get_result().detected_position.pose.position
        return result

    @classmethod
    def detect_object(cls):
        print("Detect object!...")
        detect_object_goal = detect_objectGoal()
        cls.detect_object_client.send_goal(detect_object_goal)
        the_result = cls.detect_object_client.wait_for_result()
        print("obj restult: " + str(the_result))
        return the_result
