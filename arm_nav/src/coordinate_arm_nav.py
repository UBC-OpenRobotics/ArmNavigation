#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from control_msgs.msg import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

GRIPPER_OPEN = [0.05]
GRIPPER_CLOSED = [-0.03]
GRIPPER_NEUTRAL = [0.01]

def goToPoint(Coord):
    rospy.loginfo(rospy.get_caller_id() + " I heard \n%s", Coord)

    # See Main function for comments
    arm = moveit_commander.MoveGroupCommander('arm')
    end_effector_link = arm.get_end_effector_link()
    arm.set_goal_orientation_tolerance(3)
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    arm.allow_replanning(True)
    #Didn't know how to pass arm through the the subscriber function
    #because callback was invoked by ROS

    #Creates target pose based on reference 
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = Coord.x
    target_pose.pose.position.y = Coord.y
    target_pose.pose.position.z = Coord.z + 0.1
    '''
    #Gripper Orientation 
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0
    '''
    '''
    end_pose = deepcopy(target_pose)
    
    waypoints = []
    # Set the first waypoint to be the starting pose
    wpose = deepcopy(start_pose)
    wpose.position.z += 0.1
    waypoints.append(deepcopy(wpose))
    '''
    #Setting and going to set pose
    arm.set_pose_target(target_pose, end_effector_link)
    arm.go()

    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = Coord.x
    target_pose.pose.position.y = Coord.y
    target_pose.pose.position.z = Coord.z

    arm.set_pose_target(target_pose, end_effector_link)
    arm.go()

    

def listener():

    rospy.Subscriber("arm_coordinate", Point, goToPoint)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
        
    rospy.init_node('arm_coordinate_listener', anonymous=True)
                
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