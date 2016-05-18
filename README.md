# loam_velodyne

This is a LOAM (Lidar Odometry and Mapping) ROS package for Velodyne VLP-16 3D laser scanner. This package is a simple modified copy of [loam_velodyne git repository](https://github.com/laboshinl/loam_velodyne) from **laboshinl**, which is again a modified copy of the original one release by [Ji Zhang](http://www.frc.ri.cmu.edu/~jizhang03/). His change on top of the original one is that he changed the scanRegistration.cpp to make it work with his dataset. I fixed a bug on laserOdometry.cpp to get rid of the matrix NaN error during L-M optimization step. Please cite Zhang et al.'s paper if this package is used. 

J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time. Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.([PDF](http://www.frc.ri.cmu.edu/~jizhang03/Publications/RSS_2014.pdf))([VIDEO](https://www.youtube.com/watch?feature=player_embedded&v=8ezyhTAEyHs))([SRC FILES](http://docs.ros.org/indigo/api/loam_velodyne/html/files.html))

Wiki Webpage by the Author: [http://wiki.ros.org/loam_velodyne](http://wiki.ros.org/loam_velodyne)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=8ezyhTAEyHs" target="_blank"><img src="http://img.youtube.com/vi/8ezyhTAEyHs/0.jpg" alt="LOAM back and forth" width="240" height="180" border="10" /></a>

# how to use
Here I assume you have a Catkin worksapce under ~/ros_workspace/catkin_ws.

(1) gitclone the package into your "src" folder.
+ **cd ~/ros_workspace/catkin_ws/src**
+ **git clone https://github.com/daobilige-su/loam_velodyne**

(2) compile the package
+ **cd ~/ros_workspace/catkin_ws**
+ **catkin_make**

(3) download a ROS bag file for test dataset.
+ **roscd loam_velodyne**
+ **mkdir data**
+ download [nsh_indoor_outdoor](www.frc.ri.cmu.edu/~jizhang03/Datasets/nsh_indoor_outdoor.bag), [gates_oscillating_motion](www.frc.ri.cmu.edu/~jizhang03/Datasets/gates_oscillating_motion.bag) and [laboshinl VLP-16](https://db.tt/t2r39mjZ)(unzip it) dataset into your ~/ros_workspace/catkin_ws/src/loam_velodyne/data/ folder.

(4) run the package and rosbag file
+ 1) in 1st terminal:
+ **roslaunch loam_velodyne loam_velodyne.launch**
+ 2) in 2nd terminal:
+ **roscd loam_velodyne/data/**
+ **rosbay play nsh_indoor_outdoor.bag** or **rosbay play gates_oscillating_motion.bag** or **rosbay play 2016-04-11-13-24-42.bag**(for laboshinl VLP-16 dataset)
+ (use -r 0.5 if the result look bad and it is due to the less powerful CPU, (e.g. rosbay play nsh_indoor_outdoor.bag -r 0.5))

YOU SHOULD SEE A RESULT SIMILAR TO THEIR DEMO VIDEO ([nsh_indoor_outdoor DEMO VIDEO](http://www.frc.ri.cmu.edu/~jizhang03/Videos/nsh_indoor_outdoor.mp4), [gates_oscillating_motion DEMO VIDEO](http://www.frc.ri.cmu.edu/~jizhang03/Videos/gates_oscillating_motion.mp4)) and [laboshinl VLP-16 DEMO VIDEO](https://www.youtube.com/watch?v=o1cLXY-Es54&feature=youtu.be). GOOD LUCK.

# Known Issue:
After two modifications (**laboshinl** and I), the algorithm starts diverging at some point when "gates_oscillating_motion" dataset is being processed. Needs parameters tuning?

# Original Introduction by the author:

Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar. The program contains two major threads running in parallel. An "odometry" thread computes motion of the lidar between two sweeps, at a higher frame rate. It also removes distortion in the point cloud caused by motion of the lidar. A "mapping" thread takes the undistorted point cloud and incrementally builds a map, while simultaneously computes pose of the lidar on the map at a lower frame rate. The lidar state estimation is combination of the outputs from the two threads.

If an IMU is available, the orientation (integrated from angular rate) and acceleration measurements are used to deal with general motion of the lidar, while the program takes care of the linear motion.

The program is tested on a laptop with 2.5 GHz quad cores and 6 Gib memory (the program consumes two cores). It uses a Velodyne VLP-16 lidar. 

Wiki Webpage: http://wiki.ros.org/loam_velodyne

