#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def wave():

    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('wave_1')
    arm.go()
    # Start the arm in the "resting" pose stored in the SRDF file
    arm.set_named_target('wave_2')
    arm.go()

if __name__ == '__main__':

    try:
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
            
        rospy.init_node('arm_voice_command_listener')
                    
        # Initialize the move group for the right arm
        arm = moveit_commander.MoveGroupCommander('arm')

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

            # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.005)
        
        while not rospy.is_shutdown():
            wave()
    except rospy.ROSInterruptException:
        pass
