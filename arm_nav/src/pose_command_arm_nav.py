#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def wave():
    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('wave_1')
    arm.go()
    arm.set_named_target('wave_2')
    arm.go()

def resting():
    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('resting')
    arm.go()

def carry():
    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('grab')
    arm.go()
    rospy.sleep(4)
    arm.set_named_target('carry')
    arm.go()
    rospy.sleep(2)

def dropoff():
    arm.set_named_target('grab')
    arm.go()
    rospy.sleep(2)
    arm.set_named_target('dropoff')
    arm.go()
    rospy.sleep(2)

def pose(pose_command):
    pose = pose_command.data

    rospy.loginfo("I heard %s",pose)

    if pose == "wave":
        wave()
    elif pose == "stop":
        resting()
    elif pose == "pick up":
        carry()
    elif pose == "drop off":
        dropoff()
    

def listener():
    rospy.Subscriber("arm_pose", String, pose)
    rospy.spin()

if __name__ == '__main__':
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
        
    rospy.init_node('arm_pose_command')

    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('arm')

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.005)

    listener()

