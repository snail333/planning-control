#!/usr/bin/env python3
#license removed for brevity

import rospy
from std_msgs.msg import String
from tkinter import *
from tkinter import ttk

class rospythontest:
    def __init__(self):
        pass

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
    
    def listener(self):
        rospy.Subscriber("chatter", String, self.callback, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rospython_node')
    #rospythontest_obj = rospythontest()
    root = Tk()
    root.title("control_test")
    rospythontest(root)
    root.mainloop()
    try:
        rospythontest_obj.listener()
    except rospy.ROSInterruptException:
        pass
