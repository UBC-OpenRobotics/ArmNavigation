#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from control_msgs.msg import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

GRIPPER_OPEN = [0.3]
GRIPPER_CLOSED = [-0.3]
GRIPPER_NEUTRAL = [0.028]

def controlGripper(gripper_command):
    rospy.loginfo(rospy.get_caller_id() +  "\n" + gripper_command.data)
    gripper_cmd = gripper_command.data.lower()

    if gripper_cmd == "open":
        gripper.set_joint_value_target(GRIPPER_OPEN)
    elif gripper_cmd == "close":
        gripper.set_joint_value_target(GRIPPER_CLOSED)
    elif gripper_cmd == "neutral":
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)

    gripper.go()


def listener():

    #rospy.init_node('arm_coordinate_listener', anonymous=True)

    rospy.Subscriber("gripper_state", String, controlGripper)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
        
    rospy.init_node('gripper_controller', anonymous=True)
    
# Initialize the move group for the right arm
    gripper = moveit_commander.MoveGroupCommander('gripper')

    '''
    # Get the name of the end-effector link
    end_effector_link = arm.get_end_effector_link()
    rospy.loginfo("The end effector link is: " + str(end_effector_link))
    '''

    listener()