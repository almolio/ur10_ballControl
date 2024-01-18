#!/usr/bin/env python3

"""
This ros node simulates a position for red and blue balls and adds 
visualizations in rviz.
Team: RedBallBlueBall
Team members: Benoit Auclair; Thien Le; Leonardo Oliveira
Date: February 18, 2023
"""

import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import tf
# from ball_position_sim.msg import Position_velocity

class BallPublisher:

    def __init__(self):

        ### timing ###
        self.frequency = 15  # [Hz]
        self.dt = 1 / self.frequency  # [s]
        self.rate = rospy.Rate(self.frequency)  # timing object
        self.time_stamp = 0

        ### model of red/blue balls ###
        self.nb_ball = 1 # nb blue/red balls
        self.blue_position = np.zeros([self.nb_ball,3])
        self.red_position = np.zeros([self.nb_ball,3])
        self.blue_velocity = np.zeros([self.nb_ball,3])
        self.red_velocity = np.zeros([self.nb_ball,3])

        ### safety zone
        self.zone_dim_ = 1.2 # radius of safety zone
        
        ### publishers ###
        # publisher of blue balls
        self.blue_array_pub = rospy.Publisher("/blue_ball_marker_array", MarkerArray,
                                                queue_size=1)  # queue_size=1 => only the newest map available

        self.blue_odom_pub = rospy.Publisher("/blueball/state/filtered", Odometry,
                                                queue_size=1)  # queue_size=1 => only the newest map available

        # publisher of red balls
        self.red_array_pub = rospy.Publisher("/red_ball_marker_array", MarkerArray,
                                                queue_size=1)  # queue_size=1 => only the newest map available
        
        self.red_odom_pub = rospy.Publisher("/redball/state/filtered", Odometry,
                                                queue_size=1)  # queue_size=1 => only the newest map available
        
        # publisher of safety zone
        self.zone_pub = rospy.Publisher("/zone_marker", Marker,
                                                queue_size=1)  # queue_size=1 => only the newest map available
        
        # messages used for either ball type
        self.ball_marker_array_msg = MarkerArray()
        self.ball_pos_vel_msg = Point()


    def run(self):
        """
        Main loop of class.
        @param: self
        @result: runs the step function.
        """
        while not rospy.is_shutdown():

            self.step()

            # sleep to selected frequency
            self.rate.sleep()
            # print("publishing blue ball *****")


    def step(self):
        """
        Perform an iteration.
        @param: self
        @result: publication of ball positions.
        """
        # print("ball being displayed")
        
        self.time_stamp = rospy.get_rostime()
        self.ball_position_velocity()

        # visualizations
        # self.publish_ball_markers(self.blue_array_pub, self.blue_position, [0.0,0.0,1.0])
        self.publish_ball_markers(self.red_array_pub, self.red_position, [1.0,0.0,0.0])
        self.publish_zone_marker(self.zone_pub, 2 * self.zone_dim_)

        # for simulation of tracking
        # self.publish_ball_odometry(self.blue_odom_pub, self.blue_position, self.blue_velocity)
        self.publish_ball_odometry(self.red_odom_pub, self.red_position, self.red_velocity)


    def publish_ball_odometry(self, ball_pub, ball_position, ball_velocity):
        """
        Publish marker arrays or red/blue balls.
        @param: self
        @result: publication of ball positions.
        """

        for i_ball in range(1): #range(self.nb_ball):
        
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            vec_norm = ball_velocity[i_ball,:] / np.linalg.norm(ball_velocity[i_ball,:])
            pitch = 0
            yaw = -np.arcsin(vec_norm[1])
            roll = 0
            odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "world"

            # set the psoe
            point=Point(ball_position[i_ball,0], ball_position[i_ball,1], ball_position[i_ball,2])
            odom.pose.pose = Pose(point, Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            vel = Vector3(ball_velocity[i_ball,0], ball_velocity[i_ball,1], ball_velocity[i_ball,2])
            odom.twist.twist = Twist(vel, Vector3(0, 0, 0))

            # publish the message
            ball_pub.publish(odom)


    def ball_position_velocity(self):
        """
        Simulate ball positions.
        @param: self
        @result: publication of ball positions.
        """
        ### BLUE BALLS ###
        # definition of circle
        f1=0.03
        x_center = 0.85
        y_center = 0.0
        z_mean = 0.6
        z_amp = 0
        radius = 0.35

        for i_blue in range(self.nb_ball):
            theta = 2 * np.pi * f1 * (rospy.get_time() * (1 - i_blue/2))
            x = x_center + radius * np.cos(theta)
            y = y_center + radius * np.sin(theta)
            z = z_mean + z_amp * np.sin(theta) + i_blue/2
            self.blue_position[i_blue, :] = [x,y,z]

            
            vx = -2 * np.pi * f1 * radius * np.sin(theta)
            vy = 2 * np.pi * f1 * radius * np.cos(theta)
            vz = 2 * np.pi * f1 * z_amp * np.cos(theta)
            self.blue_velocity[i_blue, :] = [vx,vy,vz]

        ### RED BALLS ###
        # definition of circle
        f1=0.03
        x_center = 0.85
        y_center = 0.0
        z_mean = 0.6
        z_amp = 0
        radius = 0.35

        for i_red in range(self.nb_ball):
            theta = 2 * np.pi * f1 * (rospy.get_time() * (1 - i_red/2))
            x = x_center + radius * np.cos(theta)
            y = y_center + radius * np.sin(theta)
            z = z_mean + z_amp * np.sin(theta) + i_red/2
            self.red_position[i_red, :] = [x,y,z]

            vx = -2 * np.pi * f1 * radius * np.sin(theta)
            vy = 2 * np.pi * f1 * radius * np.cos(theta)
            vz = 2 * np.pi * f1 * z_amp * np.cos(theta)
            self.blue_velocity[i_blue, :] = [vx,vy,vz]

    def publish_zone_marker(self, zone_pub, zone_dim):
        """
        Publish marker arrays of red/blue balls.
        @param: self
        @result: publication of ball positions.
        """
        # add to the marker array all features deemed visible
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.time_stamp
        marker.id = 1
        marker.type = np.int(2)  # display marker as spheres
        marker.action = np.int(0)
        marker.lifetime = rospy.Duration.from_sec(1)

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = zone_dim
        marker.scale.y = zone_dim
        marker.scale.z = zone_dim

        marker.color.r = 0
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.35

        zone_pub.publish(marker)


    def publish_ball_markers(self, ball_pub, ball_position, ball_color):
        """
        Publish marker arrays of red/blue balls.
        @param: self
        @result: publication of ball positions.
        """

        counter = 0
        ball_size = 0.15
        self.ball_marker_array_msg.markers = []

        for i_ball in range(self.nb_ball):
        
            # add to the marker array all features deemed visible
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.time_stamp
            marker.id = counter
            marker.type = np.int(2)  # display marker as spheres
            marker.action = np.int(0)
            marker.lifetime = rospy.Duration.from_sec(1)

            marker.pose.position.x = ball_position[i_ball, 0]
            marker.pose.position.y = ball_position[i_ball, 1]
            marker.pose.position.z = ball_position[i_ball, 2]

            # print("marker x position : ", marker.pose.position.x)

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = ball_size
            marker.scale.y = ball_size
            marker.scale.z = ball_size

            marker.color.r = ball_color[0]
            marker.color.g = ball_color[1]
            marker.color.b = ball_color[2]
            marker.color.a = 1.0

            self.ball_marker_array_msg.markers.append(marker)
            counter+=1

        ball_pub.publish(self.ball_marker_array_msg)



if __name__ == '__main__':
    # initialize node and name it
    rospy.init_node("BallPositionSim") 

    # go to class that provides all the functionality
    try:
        ballPublisher = BallPublisher()
        ballPublisher.run()
    except rospy.ROSInterruptException:
        pass

