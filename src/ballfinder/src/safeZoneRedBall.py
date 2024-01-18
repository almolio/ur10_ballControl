#!/usr/bin/env python
# Create a F from the red ball 

import rospy
import numpy as np 
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import sys

class safeZoneRedBall: 
    def __init__(self):
        ## Define safe zone: 
        self.safe_dis_from_base = 0.75 #meter

        ## Subcribe ball position 
        self.sub_redball = rospy.Subscriber("/redball/point", PointStamped, self.callback_ball_position)
        self.pub_selectball = rospy.Publisher("/trackRedBall", Odometry, queue_size= 1)

    def callback_ball_position(self, point):
        ## Calculate the distance of ball to base in the call back         
        x = point.point.x
        y = point.point.y
        z = point.point.z
        ball_distance_from_base = (x**2 + y**2 + z**2)**0.5
        if ball_distance_from_base <= self.safe_dis_from_base: 
            self.publish_odometry(x, y, z)
        else: 
            # self.publish_default()
            pass


    def ball_prioritize(self):
        pass
    
    def publish_odometry(self, x, y , z):  
        trackBall = Odometry()

        trackBall.header.child_frame_id = "trackRedBall"
        trackBall.pose.position.x = x
        trackBall.pose.position.y = y
        trackBall.pose.position.z = z

        trackBall.pose.orientation.x = 0
        trackBall.pose.orientation.y = 0
        trackBall.pose.orientation.z = 0  

        self.pub_selectball.publish(trackBall)


def main(args):
    rospy.init_node('TrackRedBall', anonymous=True)
    trackBall = safeZoneRedBall()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



if __name__ == '__main__':
    main(sys.argv)
