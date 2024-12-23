#!/usr/bin/env python3

import rospy 

def main(): 
    rospy.init_node('turtle_trajectory', anonymous=True)
    rospy.loginfo('Turtle trajectory initialized')
    rospy.spin() #keep the node running 


if __name__ == "__main__":
    main() 


