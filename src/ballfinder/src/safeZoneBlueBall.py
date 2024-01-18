#!/usr/bin/env python
# Touch the closest one first ( Prioritize ) 

import rospy
import numpy as np 
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import sys

class safeZoneBlueBall: 
    def __init__(self):
        ## Define safe zone: 
        self.safe_dis_from_base = 0.75 #meter

        ## Subcribe ball position 
        self.sub_blueball = rospy.Subscriber("/blueball/point", PointStamped, self.callback_blueball_position)
        self.pub_selectball = rospy.Publisher("/trackBlueBall", Odometry, queue_size= 1)
        self.blueballs_pos = PointStamped()


    def callback_blueball_position(self, point):
        ## Calculate the distance of ball to base in the call back         
        x = point.point.x
        y = point.point.y
        z = point.point.z
        ball_distance_from_base = (x**2 + y**2 + z**2)**0.5
        if ball_distance_from_base <= self.safe_dis_from_base: 
            self.publish_odometry(x, y, z)
        else: 
            self.publish_default()

    def publish_default(self):
        # Send to home with arm straight up 
        trackBall = Odometry()

        trackBall.header.child_frame_id = "trackBlueBall"
        trackBall.pose.position.x = 0
        trackBall.pose.position.y = 0.256141
        trackBall.pose.position.z = 1.4273

        trackBall.pose.orientation.x = 0.0
        trackBall.pose.orientation.y = 0.0
        trackBall.pose.orientation.z = 1.5707963 

        self.pub_selectball.publish(trackBall)


    def ball_prioritize(self):
        pass
    
    def publish_odometry(self, x, y , z):  
        trackBall = Odometry()

        trackBall.header.child_frame_id = "trackBlueBall"
        trackBall.pose.position.x = x
        trackBall.pose.position.y = y
        trackBall.pose.position.z = z

        trackBall.pose.orientation.x = 0
        trackBall.pose.orientation.y = 0
        trackBall.pose.orientation.z = 0  

        self.pub_selectball.publish(trackBall)


def main(args):
    rospy.init_node('TrackBlueBall', anonymous=True)
    trackBall = safeZoneBlueBall()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



if __name__ == '__main__':
    main(sys.argv)
