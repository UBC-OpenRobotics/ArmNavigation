#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

'''
def pickup():
    x=0
    y=0.2
    z=0.35
    pub = rospy.Publisher('arm_coordinate', Point, queue_size=10)
    pub.publish(float(x),float(y),float(z))
'''

def doAction(voice_command):

    pub = rospy.Publisher('arm_pose', String, queue_size=10)
    
    command = set(voice_command.data.lower().split())

    rospy.loginfo("I heard %s",command)

    if not set(['wave', 'goodbye', 'waving']).isdisjoint(command):
        pub.publish("wave")
    elif not set(["stop","resting"]).isdisjoint(command):
        pub.publish("stop")
    elif not set(["serve","handoff","pick up","carry", "pick","grab"]).isdisjoint(command):
        pub.publish("pick up")
        

def listener():
    rospy.Subscriber("grammar_data", String, doAction)
    rospy.spin()

if __name__ == '__main__':

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
        
    rospy.init_node('arm_voice_command')

    '''                
    # Initialize the move group for the right arm
    arm = moveit_commander.MoveGroupCommander('arm')

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.005)
    '''
    
    listener()
