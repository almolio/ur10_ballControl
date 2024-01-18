## Why is there so many folder? 
Matlab folder contains the model which we spend extensive time on. This is a working model. 
The bulk of project is in the src folder 
1. arucoposition -- this package grab the pose estimation of the Aruco in relation to the camera. This package also contain the fixed transformation between the aruco and robot base. This is written in python ;) because we could. Also, we wanted to see how everything works together. Additionally, since the calibration approach we took only looked at the the transformation for the first few frame, we did not need the speed of cpp. This node stay running during robot operation to broadcast previously mentioned tf frames. 
2. ball_position_sim -- development package to simulate a ball moving in a circular motion. This is so we could visualize and test our controller's ability to perform its task before applying to the real robot. This is cabaple of simulation both multiple red and blue balls. We also wrote this in Python so that we could quickly change the ball speed and trajectory during testing, without the compile time. 
3. iai_kinect2_opencv4 -- interface between the kinect 2 and ros 
4. openni2_camera/rgbd_launch -- interface betwwen the asus and ros
5. state_filter -- simple weighted moving average function to smooth the ball trajectory during real robot operation. 
6. ballfinder -- this is the bulk of the visual perception. It contain the image processing pipeline, include filter. It broadcast the ball position in space to the controller. 
7. bluecontroller -- this is our controller. It intergrates all the necessary messages from the visual system, and output the final command to the robot. Contain joint space and task space controllers. Further details about the controller can be view in our report ;). The blue controller launch file launches all the nodes it needs to run. 
8. The rest are course provided materials. We modifed the launch file to also run out blue controller

## Operation 
The robot can be run by launching the tum_ics_ur10_bringup bringUR10.launch 
This should launch Rviz visualization with the red and blue balls and well as the controller 
