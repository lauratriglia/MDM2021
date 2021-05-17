"""
 author: Laura Triglia
 
 release date: 17/05/2021
  
"""
import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(5)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            print("Robot is going to the position")
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Robot stopping")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
