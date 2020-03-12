#!/usr/bin/env python
import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import math
from Tkinter import *

main = Tk()

coords = {"x":0,"y":0,"x2":0,"y2":0}
# keep a reference to all lines by keeping them in a list 
lines = []

global armNum
armNum = 0

class Arm:
    def __init__(self,armNum,shoulder,elbow,wrist,hand):
        self.armNum = armNum
        self.shoulder = shoulder
        self.elbow = elbow
        self.wrist = wrist
        self.hand = hand

def click(e):
    a1.armNum+=1
    # define start point for line
    coords["x"] = e.x
    coords["y"] = e.y

    # create a line on this point and store it in the list
    lines.append(canvas.create_line(coords["x"],coords["y"],coords["x"],coords["y"]))

def drag(e):
    # update the coordinates from the event
    coords["x2"] = e.x
    coords["y2"] = e.y

    # Change the coordinates of the last created line to the new coordinates
    canvas.coords(lines[-1], coords["x"],coords["y"],coords["x2"],coords["y2"])
    computeAngle()

def clearCanvas():
    canvas.delete("all")
    a1.armNum = 0

    arm.set_named_target('resting')
    arm.go()
    rospy.sleep(2)

def computeAngle():
    if (coords["x2"]-coords["x"]>0):
        angle = math.atan((float(coords["y"])-coords["y2"])/(coords["x2"]-coords["x"]))
        armPositions(angle-(math.pi/2))
        print "angle: ", (angle-(math.pi/2))
    else:
        angle = math.atan((float(coords["y"])-coords["y2"])/(coords["x2"]-coords["x"]))
        armPositions(angle+math.pi)
        print "angle: ", angle+(math.pi/2)
    
def armPositions(dangle):
    '''
    if (a1.armNum-1 == 0):
        a1.shoulder = angle
        print a1.shoulder'''
    if (a1.armNum-1 == 1):    
        a1.elbow = dangle
        print "elbow angle: ", a1.elbow
    elif (a1.armNum-1 == 2):
        a1.wrist = -(a1.elbow - dangle) 
        print "wrist angle: ", -a1.elbow + dangle 
    elif (a1.armNum-1 == 3):
        a1.hand = -a1.wrist + dangle
        print "hand angle: ", -a1.wrist + dangle
    elif(dangle==100):
        joint_positions=[a1.shoulder,a1.elbow,a1.wrist,a1.hand,0]
        # Set the arm's goal configuration to the be the joint positions
        arm.set_joint_value_target(joint_positions)
        # Plan and execute the motion
        arm.go()
        rospy.sleep(1)
    print ("ap has run")
    
def goPos():
    a1.armNum+=1
    armPositions(100)

if __name__ == '__main__':

    canvas = Canvas(main, bg="white", width=600, height=400)
    canvas.pack()
    canvas.bind("<ButtonPress-1>", click)
    canvas.bind("<B1-Motion>", drag) 
    clearButton = Button(main, text="New pose", command=clearCanvas)
    goButton = Button(main, text="Go", command=goPos)
    clearButton.pack()
    goButton.pack()

    a1 = Arm(0,0,0,0,0)

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
    rospy.sleep(1)

    joint_positions=[a1.shoulder,0,0,0,a1.hand]
    # Set the arm's goal configuration to the be the joint positions
    arm.set_joint_value_target(joint_positions)
    # Plan and execute the motion
    arm.go()
    rospy.sleep(1)

    # Set the start state to the current state
    arm.set_start_state_to_current_state()

    main.mainloop()
        

    