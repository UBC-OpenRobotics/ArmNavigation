#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import curses

def main(stdscr):

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

    # do not wait for input when calling getch
    stdscr.nodelay(1)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

     # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.05)

    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('resting')
    arm.go()
    rospy.sleep(2)

    while True:
        # get keyboard input, returns -1 if none available
        c = stdscr.getch()
        if c != -1:
            # print numeric value
            stdscr.addstr(str(c) + ' ')

            #shifts arm in an axis by a certai value
            if c==curses.KEY_LEFT:
                arm.shift_pose_target(1, -0.05, end_effector_link)
                arm.go()
            if c==curses.KEY_RIGHT:
                arm.shift_pose_target(1, 0.05, end_effector_link)
                arm.go()
            if c==curses.KEY_UP:
                arm.shift_pose_target(3, 0.05, end_effector_link)
                arm.go()
            if c==curses.KEY_DOWN:
                arm.shift_pose_target(3, -0.05, end_effector_link)
                arm.go()
            #if c==curses.ord('z'):
                arm.shift_pose_target(2, 0.05, end_effector_link)
                arm.go()
            #if c==curses.ord('x'):
                arm.shift_pose_target(2, -0.05, end_effector_link)
                arm.go()

            stdscr.refresh()
            # return curser to start position
            stdscr.move(0, 0)

if __name__ == '__main__':
    curses.wrapper(main)