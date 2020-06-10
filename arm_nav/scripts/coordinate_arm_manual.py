#!/usr/bin/env python
import rospy
import Tkinter as tk
from geometry_msgs.msg import Point
from std_msgs.msg import String

root= tk.Tk()

#Widget components
canvas1 = tk.Canvas(root, width = 400, height = 400)
canvas1.pack()

label1 = tk.Label(root, text='x coordinate:')
label1.config(font=('helvetica', 10))
canvas1.create_window(200, 30, window=label1)

entry1 = tk.Entry (root) 
canvas1.create_window(200, 70, window=entry1)

label2 = tk.Label(root, text='y coordinate:')
label2.config(font=('helvetica', 10))
canvas1.create_window(200, 110, window=label2)

entry2 = tk.Entry (root) 
canvas1.create_window(200, 150, window=entry2)

label3 = tk.Label(root, text='z coordinate:')
label3.config(font=('helvetica', 10))
canvas1.create_window(200, 190, window=label3)

entry3 = tk.Entry (root) 
canvas1.create_window(200, 230, window=entry3)
    
def talker(x, y, z):
    pub = rospy.Publisher('arm_coordinate', Point, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        rospy.loginfo("\nx:"+x+"\ny:"+y+"\nz:"+z)

        #Publishes Point message
        pub.publish(float(x),float(y),float(z))
        rate.sleep()

        #widget 
        root.mainloop()

#Gets the coordinates from the text boxes when 'Go to Point' button is pressed
def getCoordinate ():  
    x = entry1.get()
    y = entry2.get()
    z = entry3.get()

    talker(x, y, z)

button1 = tk.Button(text='Go to Point', command=getCoordinate)
canvas1.create_window(200, 270, window=button1)

def OpenGripper():
    pub1 = rospy.Publisher('gripper_state', String, queue_size=10)
    pub1.publish("open")

button2 = tk.Button(text='Open', command=OpenGripper)
canvas1.create_window(100, 330, window=button2)

def CloseGripper():
    pub1 = rospy.Publisher('gripper_state', String, queue_size=10)
    pub1.publish("close")

button3 = tk.Button(text='Close', command=CloseGripper)
canvas1.create_window(200, 330, window=button3)

def NeutralGripper():
    pub1 = rospy.Publisher('gripper_state', String, queue_size=10)
    pub1.publish("neutral")

button4 = tk.Button(text='Neutral', command=NeutralGripper)
canvas1.create_window(300, 330, window=button4)

if __name__ == '__main__':

    try:
        rospy.init_node('arm_coordinate_manual', anonymous=True)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass