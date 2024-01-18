rosrun kinect2_bridge kinect2_bridge _fps_limit:=2
mkdir ~/kinect_cal_data; cd ~/kinect_cal_data
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 record color
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 calibrate color
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 record ir
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 calibrate ir
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 record sync
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 calibrate sync
rosrun kinect2_calibration kinect2_calibration chess7x9x0.025 calibrate depth
roscd kinect2_bridge/data; mkdir 000689562647
