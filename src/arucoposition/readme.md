roslaunch kinect2_bridge kinect2_bridge.launch
rosrun kinect2_viewer kinect2_viewer sd cloud
rosrun arucoposition arucoBroadcast.py 



## TODO: 
Make rviz file and launch file


We're now pulling camera config from sd and apply it to hd