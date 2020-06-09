#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from Tkinter import *

main = Tk()

def leftKey(event):
    print "Left key pressed"
    arm.shift_pose_target(0, -0.05, end_effector_link)
    arm.go()
    print "Shifted Left"
    #rospy.sleep(1)

def rightKey(event):
    print "Right key pressed"
    arm.shift_pose_target(0, 0.05, end_effector_link)
    arm.go()
    print "Shifted Right"
    #rospy.sleep(1)

def upKey(event):
    print "Up key pressed"
    arm.shift_pose_target(2, 0.05, end_effector_link)
    arm.go()
    print "Shifted Up"
    #rospy.sleep(1)
    
def downKey(event):
    print "Down key pressed"
    arm.shift_pose_target(2, -0.05, end_effector_link)
    arm.go()
    print "Shifted Down"
    #rospy.sleep(1)

def forwardKey(event):
    print "F key pressed"
    arm.shift_pose_target(1, 0.05, end_effector_link)
    arm.go()
    print "Shifted Forward"
    #rospy.sleep(1)

def backwardKey(event):
    print "B key pressed"
    arm.shift_pose_target(1, -0.05, end_effector_link)
    arm.go()
    print "Shifted Backwards"
    #rospy.sleep(1)

if __name__ == '__main__':

    frame = Frame(main, width=100, height=100)
    frame.bind('<Left>', leftKey)
    frame.bind('<Right>', rightKey)
    frame.bind('<Up>', upKey)
    frame.bind('<Down>', downKey)
    frame.bind('f', forwardKey)
    frame.bind('b', backwardKey)
    frame.focus_set()
    frame.pack()

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

     # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.005)

    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('resting')
    arm.go()
    rospy.sleep(2)

    '''
    # Set the target pose.  This particular pose has the gripper oriented horizontally
    # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
    # the center of the robot base.
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = 0.20
    target_pose.pose.position.y = -0.1
    target_pose.pose.position.z = 0.85
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0 
    '''   

    # Set the start state to the current state
    arm.set_start_state_to_current_state()

    arm.shift_pose_target(2, 0.05, end_effector_link)
    arm.go()

    main.mainloop()
        