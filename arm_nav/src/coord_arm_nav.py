#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def goToPoint(Coord):
    rospy.loginfo(rospy.get_caller_id() + "I heard x: %d, y: %d, z: %d", Coord.x, Coord.y, Coord.z)

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_footprint'
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = Coord.x
    target_pose.pose.position.y = Coord.y
    target_pose.pose.position.z = Coord.z
    
    ''' Gripper Orientation 
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0
    '''

    right_arm.set_pose_target(target_pose, end_effector_link)

    right_arm.go()
    
def listener():

    rospy.init_node('arm_coord_listener', anonymous=True)

    rospy.Subscriber("arm_coord",Point, goToPoint)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
        
    rospy.init_node('moveit_demo')
                
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('arm')
                
    # Get the name of the end-effector link
    end_effector_link = arm.get_end_effector_link()
                        
    # Set the reference frame for pose targets
    reference_frame = 'base_link'
        
    # Set the right arm reference frame accordingly
    arm.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('resting')
    arm.go()

    rospy.loginfo("Resting in Initial Position")

    listener()