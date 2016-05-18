00001 #include <math.h>
00002 #include <time.h>
00003 #include <stdio.h>
00004 #include <stdlib.h>
00005 #include <ros/ros.h>
00006 
00007 #include <nav_msgs/Odometry.h>
00008 #include <sensor_msgs/Imu.h>
00009 #include <sensor_msgs/PointCloud2.h>
00010 
00011 #include <tf/transform_datatypes.h>
00012 #include <tf/transform_broadcaster.h>
00013 
00014 #include <opencv/cv.h>
00015 #include <opencv2/highgui/highgui.hpp>
00016 
00017 #include <pcl_conversions/pcl_conversions.h>
00018 #include <pcl/point_cloud.h>
00019 #include <pcl/point_types.h>
00020 #include <pcl/filters/voxel_grid.h>
00021 #include <pcl/kdtree/kdtree_flann.h>
00022 
00023 const double PI = 3.1415926;
00024 const double rad2deg = 180 / PI;
00025 const double deg2rad = PI / 180;
00026 
00027 float transformSum[6] = {0};
00028 float transformIncre[6] = {0};
00029 float transformMapped[6] = {0};
00030 float transformBefMapped[6] = {0};
00031 float transformAftMapped[6] = {0};
00032 
00033 ros::Publisher *pubLaserOdometry2Pointer = NULL;
00034 tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
00035 nav_msgs::Odometry laserOdometry2;
00036 tf::StampedTransform laserOdometryTrans2;
00037 
00038 void transformAssociateToMap()
00039 {
00040   float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
00041            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
00042   float y1 = transformBefMapped[4] - transformSum[4];
00043   float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
00044            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
00045 
00046   float x2 = x1;
00047   float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
00048   float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;
00049 
00050   transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
00051   transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
00052   transformIncre[5] = z2;
00053 
00054   float sbcx = sin(transformSum[0]);
00055   float cbcx = cos(transformSum[0]);
00056   float sbcy = sin(transformSum[1]);
00057   float cbcy = cos(transformSum[1]);
00058   float sbcz = sin(transformSum[2]);
00059   float cbcz = cos(transformSum[2]);
00060 
00061   float sblx = sin(transformBefMapped[0]);
00062   float cblx = cos(transformBefMapped[0]);
00063   float sbly = sin(transformBefMapped[1]);
00064   float cbly = cos(transformBefMapped[1]);
00065   float sblz = sin(transformBefMapped[2]);
00066   float cblz = cos(transformBefMapped[2]);
00067 
00068   float salx = sin(transformAftMapped[0]);
00069   float calx = cos(transformAftMapped[0]);
00070   float saly = sin(transformAftMapped[1]);
00071   float caly = cos(transformAftMapped[1]);
00072   float salz = sin(transformAftMapped[2]);
00073   float calz = cos(transformAftMapped[2]);
00074 
00075   float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
00076             - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00077             - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00078             - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00079             - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
00080   transformMapped[0] = -asin(srx);
00081 
00082   float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00083                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00084                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00085                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
00086                + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
00087   float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00088                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
00089                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00090                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00091                + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
00092   transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), crycrx / cos(transformMapped[0]));
00093   
00094   float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
00095                - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
00096                - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
00097                + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
00098                - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
00099                + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
00100                + calx*cblx*salz*sblz);
00101   float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
00102                - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
00103                + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
00104                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
00105                + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
00106                - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
00107                - calx*calz*cblx*sblz);
00108   transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), crzcrx / cos(transformMapped[0]));
00109 
00110   x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
00111   y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
00112   z1 = transformIncre[5];
00113 
00114   x2 = x1;
00115   y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
00116   z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;
00117 
00118   transformMapped[3] = transformAftMapped[3] 
00119                      - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
00120   transformMapped[4] = transformAftMapped[4] - y2;
00121   transformMapped[5] = transformAftMapped[5] 
00122                      - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
00123 }
00124 
00125 void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
00126 {
00127   double roll, pitch, yaw;
00128   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
00129   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
00130 
00131   transformSum[0] = -pitch;
00132   transformSum[1] = -yaw;
00133   transformSum[2] = roll;
00134 
00135   transformSum[3] = laserOdometry->pose.pose.position.x;
00136   transformSum[4] = laserOdometry->pose.pose.position.y;
00137   transformSum[5] = laserOdometry->pose.pose.position.z;
00138 
00139   transformAssociateToMap();
00140 
00141   geoQuat = tf::createQuaternionMsgFromRollPitchYaw
00142             (transformMapped[2], -transformMapped[0], -transformMapped[1]);
00143 
00144   laserOdometry2.header.stamp = laserOdometry->header.stamp;
00145   laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
00146   laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
00147   laserOdometry2.pose.pose.orientation.z = geoQuat.x;
00148   laserOdometry2.pose.pose.orientation.w = geoQuat.w;
00149   laserOdometry2.pose.pose.position.x = transformMapped[3];
00150   laserOdometry2.pose.pose.position.y = transformMapped[4];
00151   laserOdometry2.pose.pose.position.z = transformMapped[5];
00152   pubLaserOdometry2Pointer->publish(laserOdometry2);
00153 
00154   laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
00155   laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
00156   laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
00157   tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);
00158 }
00159 
00160 void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
00161 {
00162   double roll, pitch, yaw;
00163   geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
00164   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
00165 
00166   transformAftMapped[0] = -pitch;
00167   transformAftMapped[1] = -yaw;
00168   transformAftMapped[2] = roll;
00169 
00170   transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
00171   transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
00172   transformAftMapped[5] = odomAftMapped->pose.pose.position.z;
00173 
00174   transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
00175   transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
00176   transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;
00177 
00178   transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
00179   transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
00180   transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
00181 }
00182 
00183 int main(int argc, char** argv)
00184 {
00185   ros::init(argc, argv, "transformMaintenance");
00186   ros::NodeHandle nh;
00187 
00188   ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
00189                                      ("/laser_odom_to_init", 5, laserOdometryHandler);
00190 
00191   ros::Subscriber subOdomAftMapped = nh.subscribe<nav_msgs::Odometry> 
00192                                      ("/aft_mapped_to_init", 5, odomAftMappedHandler);
00193 
00194   ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
00195   pubLaserOdometry2Pointer = &pubLaserOdometry2;
00196   laserOdometry2.header.frame_id = "/camera_init";
00197   laserOdometry2.child_frame_id = "/camera";
00198 
00199   tf::TransformBroadcaster tfBroadcaster2;
00200   tfBroadcaster2Pointer = &tfBroadcaster2;
00201   laserOdometryTrans2.frame_id_ = "/camera_init";
00202   laserOdometryTrans2.child_frame_id_ = "/camera";
00203 
00204   ros::spin();
00205 
00206   return 0;
00207 }
