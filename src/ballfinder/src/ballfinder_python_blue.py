import sys
import rospy
import numpy as np
import cv2
import ros_numpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import tf


FILTERING_BUFFER_SIZE = 10

COLOR_TOPIC = "/camera/rgb/image_rect_color"
DEPTH_IMAGE_TOPIC = "/camera/depth_registered/hw_registered/image_rect"
CAMERA_INFO_TOPIC = "/camera/rgb/camera_info"
BALL_TOPIC = "/trackedBall/blue"


class BallFinder:

    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_image_sub = rospy.Subscriber(COLOR_TOPIC, Image, self.callback_rgb)
        self.depth_image_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, self.callback_depth)
        self.point_pub = rospy.Publisher(BALL_TOPIC, Odometry, queue_size=1)
        self.cam_info_sub = rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo,self.infocallback)
        self.tfsub = tf.TransformListener()
        self.depth = None
        self.rgb = None
        self.camera_matrix = None
        self.filtered_position_buffer = None
        self.last_time = 0
        self.current_time = 0
        self.velocity = np.zeros(3)
        self.safe_dis_from_base = 0.75
        self.center = (0,0)

    def callback_depth(self,data):
        try:
            cv_image = ros_numpy.image.image_to_numpy(data)
            self.depth = cv_image
        except CvBridgeError as e:
            print(e)

    def infocallback(self,info):
        self.camera_matrix = np.asarray(info.K).reshape([3,3])

    def preprocess_image(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        img[:,:,0] = cv2.equalizeHist(img[:,:,0])
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR)

        # Apply gaussian blur
        img = cv2.GaussianBlur(img, (9, 9), 0)

        return img

    def segment_image(self, img):

        # Convert to HSV
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Blue
        lower_boundaries = [
            [100, 150, 20],
        ]
        upper_boundaries = [
            [130, 255, 255]
        ]

       # Create mask
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        for i in range(len(lower_boundaries)):
            lower = np.array(lower_boundaries[i], dtype=np.uint8)
            upper = np.array(upper_boundaries[i], dtype=np.uint8)
            mask += cv2.inRange(img, lower, upper)


        # Apply erosion and dilation to remove some noise
        kernel = np.ones((7,7),np.uint8)
        mask = cv2.erode(mask,kernel,iterations = 1)
        mask = cv2.dilate(mask,kernel,iterations = 1)

        return mask

    def callback_rgb(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.rgb = cv_image
        except CvBridgeError as e:
            print(e)

        # Perform color segmentationreconstruct_center_point
        red_bbox = self.get_color_image(cv_image)

        if not red_bbox:
            print("No red object found. Skipping")
            return

        # Get bbox center, draw circle and show image
        x, y, w, h = red_bbox
        self.center = (x + w//2, y + h//2)
        cv2.circle(cv_image, self.center, 5, (255, 0, 255), -1)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def post_process(self):
    
        x_, y_, z_ = self.reconstruct_center_point(self.center)

        T_ball_cam = np.eye(4)
        T_ball_cam[0:3, 3] = [x_,y_,z_]

        # target, source 
        # Camera wrt world
        now = rospy.Time.now()
        T_cam_world = tf.TransformerROS.fromTranslationRotation( 
            self.tfsub.waitForTransform("camera_rgb_optical_frame", "world",  now, rospy.Duration(5.0)))

        T_ball_world = T_ball_cam * T_cam_world

        x, y, z = T_ball_world[0:3,3]

        filt_x, filt_y, filt_z = self.filter_position(x, y, z)

        self.last_time = self.current_time
        self.current_time = rospy.get_time()

        # Careful: velocity is in camera frame!
        self.velocity = self.estimate_velocity() / (self.current_time - self.last_time)

        print(f"Found red ball at ({x}, {y}, {z})")
        print(f"Red ball velocity: {self.velocity}")

        ball_distance_from_base = (filt_x**2 + filt_y**2 + filt_z**2)**0.5
        if ball_distance_from_base <= self.safe_dis_from_base: 
            self.publish_odometry("BLUE", x, y, z, self.velocity, self.current_time)
        else: 
            self.publish_odometry("BLUE", 0, 0.256141, 1.4273, self.velocity, self.current_time)

        # Publish point


    def publish_odometry(self, name, x, y , z, velocity, time ):  
        '''use this function structure the message we need to send to the controller '''
        
        trackBall = Odometry()

        trackBall.header.stamp = time
        trackBall.header.frame_id = "world"
        trackBall.child_frame_id = name
        trackBall.pose.pose.position.x = x
        trackBall.pose.pose.position.y = y
        trackBall.pose.pose.position.z = z

        trackBall.pose.pose.orientation.x = 0
        trackBall.pose.pose.orientation.y = 0
        trackBall.pose.pose.orientation.z = 0  

        trackBall.twist.twist.linear.x = velocity(0)
        trackBall.twist.twist.linear.y = velocity(1)
        trackBall.twist.twist.linear.z = velocity(2)

        self.point_pub.publish(trackBall)
        
    def filter_position(self, x, y, z):

        xyz = np.array([x, y, z])[None, :]
        if self.filtered_position_buffer is None:
            self.filtered_position_buffer = xyz
            return
        
        # Appending current observation to circular buffer
        self.filtered_position_buffer = np.vstack([self.filtered_position_buffer, xyz])
        if self.filtered_position_buffer.shape[0] > FILTERING_BUFFER_SIZE:
            # Dropping oldest observation if the circular buffer is full
            self.filtered_position_buffer = self.filtered_position_buffer[1:]
                
        self.filtered_position = self.filtered_position_buffer.mean(axis=0)
        return self.filtered_position

    def estimate_velocity(self) -> np.ndarray:
        if self.filtered_position_buffer is None:
            return np.zeros(3)
        
        # Estimate velocity by using np.diff on the circular buffer
        velocity = np.diff(self.filtered_position_buffer, axis=0).mean(axis=0)
        return velocity
    
    def get_color_image(self, img):
        
        img = self.preprocess_image(img)
        mask = self.segment_image(img)

        # Get biggest bounding box
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_bbox = None
        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            red_bbox = x, y, w, h

        # return bounding box
        return red_bbox

    def reconstruct_center_point(self, center):

        if self.camera_matrix is None:
            print("Camera matrix not ready yet.")
            return 0,0,0
        # Since we subscribed to the rectified images, we can use the camera matrix directly
        fx = self.camera_matrix[0,0]
        fy = self.camera_matrix[1,1]
        cx = self.camera_matrix[0,2]
        cy = self.camera_matrix[1,2]

        # Get center of bounding box
        u, v = center 

        # Get depth at center using projection model
        z = self.depth[v, u]
        z = z / 1000.0     # convert to meters
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z

def shutdown_hook(): 
    cv2.destroyAllWindows()


def main(args):
    rospy.init_node('ball_finder', anonymous=True)
    bf = BallFinder()
    while not rospy.is_shutdown():
        bf.post_process()

    rospy.on_shutdown(shutdown_hook)

if __name__ == '__main__':
    main(sys.argv)
