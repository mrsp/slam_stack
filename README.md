# slam_stack
A ROS stack for g2o-based slam with Kinect Fusion and TSDF Interpolation

# Install
* cd your_ros_ws/src
* git clone --recursive https://github.com/mrsp/slam_stack.git
* cd ..
* catkin_make -DCMAKE_BUILD_TYPE=Release


# Run
* roslaunch multi_voxels multi_voxels_kfp.launch
* roslaunch key_frame_publisher key_frame_publisher_xtion.launch
* roslaunch g2o_slam g2o_slam_kfp_xtion.launch
* rosbag play vipgpu.bag
