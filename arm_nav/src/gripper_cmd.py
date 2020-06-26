#!/usr/bin/env python
import rospy, sys

from std_msgs.msg import String, Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

GRIPPER_OPEN = 1.4
GRIPPER_CLOSED = -1.6
GRIPPER_NEUTRAL = 0.0
GRIPPER_PSEUDO_CLOSED = -0.3

def controlGripper(gripper_command):

    pub = rospy.Publisher('gripper_joint/command', Float64, queue_size=10)

    rospy.loginfo(rospy.get_caller_id() +  "\n" + gripper_command.data)
    gripper_cmd = gripper_command.data.lower()

    if gripper_cmd == "open":
        pub.publish(GRIPPER_OPEN)
    elif gripper_cmd == "close":
        pub.publish(GRIPPER_CLOSED)
    elif gripper_cmd == "neutral":
        pub.publish(GRIPPER_NEUTRAL)
    elif gripper_cmd == "pseudo closed":
        pub.publish(GRIPPER_PSEUDO_CLOSED)


def listener():

    #rospy.init_node('arm_coordinate_listener', anonymous=True)

    rospy.Subscriber("gripper_state", String, controlGripper)

    rospy.loginfo("gripper listening")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
       
    rospy.init_node('gripper_controller', anonymous=True)

    listener()