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
00024 
00025 const float scanPeriod = 0.1;
00026 
00027 const int stackFrameNum = 1;
00028 const int mapFrameNum = 5;
00029 
00030 double timeLaserCloudCornerLast = 0;
00031 double timeLaserCloudSurfLast = 0;
00032 double timeLaserCloudFullRes = 0;
00033 double timeLaserOdometry = 0;
00034 
00035 bool newLaserCloudCornerLast = false;
00036 bool newLaserCloudSurfLast = false;
00037 bool newLaserCloudFullRes = false;
00038 bool newLaserOdometry = false;
00039 
00040 int laserCloudCenWidth = 10;
00041 int laserCloudCenHeight = 5;
00042 int laserCloudCenDepth = 10;
00043 const int laserCloudWidth = 21;
00044 const int laserCloudHeight = 11;
00045 const int laserCloudDepth = 21;
00046 const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
00047 
00048 int laserCloudValidInd[125];
00049 int laserCloudSurroundInd[125];
00050 
00051 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
00052 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
00053 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>());
00054 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>());
00055 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack2(new pcl::PointCloud<pcl::PointXYZI>());
00056 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack2(new pcl::PointCloud<pcl::PointXYZI>());
00057 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());
00058 //pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZI>());
00059 //pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorr(new pcl::PointCloud<pcl::PointXYZI>());
00060 //pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudProj(new pcl::PointCloud<pcl::PointXYZI>());
00061 pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
00062 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>());
00063 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZI>());
00064 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>());
00065 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>());
00066 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>());
00067 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray[laserCloudNum];
00068 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray[laserCloudNum];
00069 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray2[laserCloudNum];
00070 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray2[laserCloudNum];
00071 
00072 pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
00073 pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
00074 
00075 float transformSum[6] = {0};
00076 float transformIncre[6] = {0};
00077 float transformTobeMapped[6] = {0};
00078 float transformBefMapped[6] = {0};
00079 float transformAftMapped[6] = {0};
00080 
00081 int imuPointerFront = 0;
00082 int imuPointerLast = -1;
00083 const int imuQueLength = 200;
00084 
00085 double imuTime[imuQueLength] = {0};
00086 float imuRoll[imuQueLength] = {0};
00087 float imuPitch[imuQueLength] = {0};
00088 
00089 void transformAssociateToMap()
00090 {
00091   float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
00092            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
00093   float y1 = transformBefMapped[4] - transformSum[4];
00094   float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
00095            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
00096 
00097   float x2 = x1;
00098   float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
00099   float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;
00100 
00101   transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
00102   transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
00103   transformIncre[5] = z2;
00104 
00105   float sbcx = sin(transformSum[0]);
00106   float cbcx = cos(transformSum[0]);
00107   float sbcy = sin(transformSum[1]);
00108   float cbcy = cos(transformSum[1]);
00109   float sbcz = sin(transformSum[2]);
00110   float cbcz = cos(transformSum[2]);
00111 
00112   float sblx = sin(transformBefMapped[0]);
00113   float cblx = cos(transformBefMapped[0]);
00114   float sbly = sin(transformBefMapped[1]);
00115   float cbly = cos(transformBefMapped[1]);
00116   float sblz = sin(transformBefMapped[2]);
00117   float cblz = cos(transformBefMapped[2]);
00118 
00119   float salx = sin(transformAftMapped[0]);
00120   float calx = cos(transformAftMapped[0]);
00121   float saly = sin(transformAftMapped[1]);
00122   float caly = cos(transformAftMapped[1]);
00123   float salz = sin(transformAftMapped[2]);
00124   float calz = cos(transformAftMapped[2]);
00125 
00126   float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
00127             - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00128             - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00129             - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00130             - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
00131   transformTobeMapped[0] = -asin(srx);
00132 
00133   float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00134                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00135                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00136                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
00137                + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
00138   float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00139                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
00140                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00141                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00142                + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
00143   transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
00144                                  crycrx / cos(transformTobeMapped[0]));
00145   
00146   float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
00147                - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
00148                - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
00149                + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
00150                - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
00151                + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
00152                + calx*cblx*salz*sblz);
00153   float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
00154                - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
00155                + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
00156                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
00157                + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
00158                - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
00159                - calx*calz*cblx*sblz);
00160   transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
00161                                  crzcrx / cos(transformTobeMapped[0]));
00162 
00163   x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
00164   y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
00165   z1 = transformIncre[5];
00166 
00167   x2 = x1;
00168   y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
00169   z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
00170 
00171   transformTobeMapped[3] = transformAftMapped[3] 
00172                          - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
00173   transformTobeMapped[4] = transformAftMapped[4] - y2;
00174   transformTobeMapped[5] = transformAftMapped[5] 
00175                          - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
00176 }
00177 
00178 void transformUpdate()
00179 {
00180   if (imuPointerLast >= 0) {
00181     float imuRollLast = 0, imuPitchLast = 0;
00182     while (imuPointerFront != imuPointerLast) {
00183       if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
00184         break;
00185       }
00186       imuPointerFront = (imuPointerFront + 1) % imuQueLength;
00187     }
00188 
00189     if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
00190       imuRollLast = imuRoll[imuPointerFront];
00191       imuPitchLast = imuPitch[imuPointerFront];
00192     } else {
00193       int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
00194       float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
00195                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
00196       float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
00197                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
00198 
00199       imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
00200       imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
00201     }
00202 
00203     transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
00204     transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
00205   }
00206 
00207   //transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * transformSum[0];
00208   //transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * transformSum[2];
00209 
00210   for (int i = 0; i < 6; i++) {
00211     transformBefMapped[i] = transformSum[i];
00212     transformAftMapped[i] = transformTobeMapped[i];
00213   }
00214 }
00215 
00216 void pointAssociateToMap(pcl::PointXYZI *pi, pcl::PointXYZI *po)
00217 {
00218   float x1 = cos(transformTobeMapped[2]) * pi->x
00219            - sin(transformTobeMapped[2]) * pi->y;
00220   float y1 = sin(transformTobeMapped[2]) * pi->x
00221            + cos(transformTobeMapped[2]) * pi->y;
00222   float z1 = pi->z;
00223 
00224   float x2 = x1;
00225   float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
00226   float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
00227 
00228   po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
00229         + transformTobeMapped[3];
00230   po->y = y2 + transformTobeMapped[4];
00231   po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
00232         + transformTobeMapped[5];
00233   po->intensity = pi->intensity;
00234 }
00235 
00236 void pointAssociateTobeMapped(pcl::PointXYZI *pi, pcl::PointXYZI *po)
00237 {
00238   float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3]) 
00239            - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
00240   float y1 = pi->y - transformTobeMapped[4];
00241   float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3]) 
00242            + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
00243 
00244   float x2 = x1;
00245   float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
00246   float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
00247 
00248   po->x = cos(transformTobeMapped[2]) * x2
00249         + sin(transformTobeMapped[2]) * y2;
00250   po->y = -sin(transformTobeMapped[2]) * x2
00251         + cos(transformTobeMapped[2]) * y2;
00252   po->z = z2;
00253   po->intensity = pi->intensity;
00254 }
00255 
00256 void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
00257 {
00258   timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();
00259 
00260   laserCloudCornerLast->clear();
00261   pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);
00262 
00263   newLaserCloudCornerLast = true;
00264 }
00265 
00266 void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
00267 {
00268   timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();
00269 
00270   laserCloudSurfLast->clear();
00271   pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);
00272 
00273   newLaserCloudSurfLast = true;
00274 }
00275 
00276 void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
00277 {
00278   timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
00279 
00280   laserCloudFullRes->clear();
00281   pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
00282 
00283   newLaserCloudFullRes = true;
00284 }
00285 
00286 void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
00287 {
00288   timeLaserOdometry = laserOdometry->header.stamp.toSec();
00289 
00290   double roll, pitch, yaw;
00291   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
00292   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
00293 
00294   transformSum[0] = -pitch;
00295   transformSum[1] = -yaw;
00296   transformSum[2] = roll;
00297 
00298   transformSum[3] = laserOdometry->pose.pose.position.x;
00299   transformSum[4] = laserOdometry->pose.pose.position.y;
00300   transformSum[5] = laserOdometry->pose.pose.position.z;
00301 
00302   newLaserOdometry = true;
00303 }
00304 
00305 void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
00306 {
00307   double roll, pitch, yaw;
00308   tf::Quaternion orientation;
00309   tf::quaternionMsgToTF(imuIn->orientation, orientation);
00310   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
00311 
00312   imuPointerLast = (imuPointerLast + 1) % imuQueLength;
00313 
00314   imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
00315   imuRoll[imuPointerLast] = roll;
00316   imuPitch[imuPointerLast] = pitch;
00317 }
00318 
00319 int main(int argc, char** argv)
00320 {
00321   ros::init(argc, argv, "laserMapping");
00322   ros::NodeHandle nh;
00323 
00324   ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
00325                                             ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);
00326 
00327   ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
00328                                           ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler);
00329 
00330   ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
00331                                      ("/laser_odom_to_init", 5, laserOdometryHandler);
00332 
00333   ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> 
00334                                          ("/velodyne_cloud_3", 2, laserCloudFullResHandler);
00335 
00336   ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);
00337 
00338   ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2> 
00339                                          ("/laser_cloud_surround", 1);
00340 
00341   ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> 
00342                                         ("/velodyne_cloud_registered", 2);
00343 
00344   //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 2);
00345 
00346   //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 2);
00347 
00348   //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 2);
00349 
00350   //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 2);
00351 
00352   ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
00353   nav_msgs::Odometry odomAftMapped;
00354   odomAftMapped.header.frame_id = "/camera_init";
00355   odomAftMapped.child_frame_id = "/aft_mapped";
00356 
00357   tf::TransformBroadcaster tfBroadcaster;
00358   tf::StampedTransform aftMappedTrans;
00359   aftMappedTrans.frame_id_ = "/camera_init";
00360   aftMappedTrans.child_frame_id_ = "/aft_mapped";
00361 
00362   std::vector<int> pointSearchInd;
00363   std::vector<float> pointSearchSqDis;
00364 
00365   pcl::PointXYZI pointOri, pointSel, pointProj, coeff;
00366 
00367   cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
00368   cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
00369   cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));
00370 
00371   cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
00372   cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
00373   cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
00374 
00375   bool isDegenerate = false;
00376   cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
00377 
00378   pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;
00379   downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
00380 
00381   pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
00382   downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
00383 
00384   pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;
00385   downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
00386 
00387   for (int i = 0; i < laserCloudNum; i++) {
00388     laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
00389     laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
00390     laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
00391     laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
00392   }
00393 
00394   int frameCount = stackFrameNum - 1;
00395   int mapFrameCount = mapFrameNum - 1;
00396   ros::Rate rate(100);
00397   bool status = ros::ok();
00398   while (status) {
00399     ros::spinOnce();
00400 
00401     if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
00402         fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
00403         fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
00404         fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005) {
00405       newLaserCloudCornerLast = false;
00406       newLaserCloudSurfLast = false;
00407       newLaserCloudFullRes = false;
00408       newLaserOdometry = false;
00409 
00410       frameCount++;
00411       if (frameCount >= stackFrameNum) {
00412         transformAssociateToMap();
00413 
00414         int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
00415         for (int i = 0; i < laserCloudCornerLastNum; i++) {
00416           pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
00417           laserCloudCornerStack2->push_back(pointSel);
00418         }
00419 
00420         int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
00421         for (int i = 0; i < laserCloudSurfLastNum; i++) {
00422           pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
00423           laserCloudSurfStack2->push_back(pointSel);
00424         }
00425       }
00426 
00427       if (frameCount >= stackFrameNum) {
00428         frameCount = 0;
00429 
00430         pcl::PointXYZI pointOnYAxis;
00431         pointOnYAxis.x = 0.0;
00432         pointOnYAxis.y = 10.0;
00433         pointOnYAxis.z = 0.0;
00434         pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);
00435 
00436         int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
00437         int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
00438         int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;
00439 
00440         if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
00441         if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
00442         if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;
00443 
00444         while (centerCubeI < 3) {
00445           for (int j = 0; j < laserCloudHeight; j++) {
00446             for (int k = 0; k < laserCloudDepth; k++) {
00447               int i = laserCloudWidth - 1;
00448               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
00449               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00450               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
00451               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00452               for (; i >= 1; i--) {
00453                 laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00454                 laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
00455                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00456                 laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00457               }
00458               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00459               laserCloudCubeCornerPointer;
00460               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00461               laserCloudCubeSurfPointer;
00462               laserCloudCubeCornerPointer->clear();
00463               laserCloudCubeSurfPointer->clear();
00464             }
00465           }
00466 
00467           centerCubeI++;
00468           laserCloudCenWidth++;
00469         }
00470 
00471         while (centerCubeI >= laserCloudWidth - 3) {
00472           for (int j = 0; j < laserCloudHeight; j++) {
00473             for (int k = 0; k < laserCloudDepth; k++) {
00474               int i = 0;
00475               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
00476               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00477               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
00478               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00479               for (; i < laserCloudWidth - 1; i++) {
00480                 laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00481                 laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
00482                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00483                 laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00484               }
00485               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00486               laserCloudCubeCornerPointer;
00487               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00488               laserCloudCubeSurfPointer;
00489               laserCloudCubeCornerPointer->clear();
00490               laserCloudCubeSurfPointer->clear();
00491             }
00492           }
00493 
00494           centerCubeI--;
00495           laserCloudCenWidth--;
00496         }
00497 
00498         while (centerCubeJ < 3) {
00499           for (int i = 0; i < laserCloudWidth; i++) {
00500             for (int k = 0; k < laserCloudDepth; k++) {
00501               int j = laserCloudHeight - 1;
00502               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
00503               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00504               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
00505               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00506               for (; j >= 1; j--) {
00507                 laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00508                 laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
00509                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00510                 laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
00511               }
00512               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00513               laserCloudCubeCornerPointer;
00514               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00515               laserCloudCubeSurfPointer;
00516               laserCloudCubeCornerPointer->clear();
00517               laserCloudCubeSurfPointer->clear();
00518             }
00519           }
00520  
00521           centerCubeJ++;
00522           laserCloudCenHeight++;
00523         } 
00524 
00525         while (centerCubeJ >= laserCloudHeight - 3) {
00526           for (int i = 0; i < laserCloudWidth; i++) {
00527             for (int k = 0; k < laserCloudDepth; k++) {
00528               int j = 0;
00529               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
00530               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00531               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
00532               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00533               for (; j < laserCloudHeight - 1; j++) {
00534                 laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00535                 laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
00536                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00537                 laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
00538               }
00539               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00540               laserCloudCubeCornerPointer;
00541               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00542               laserCloudCubeSurfPointer;
00543               laserCloudCubeCornerPointer->clear();
00544               laserCloudCubeSurfPointer->clear();
00545             }
00546           }
00547 
00548           centerCubeJ--;
00549           laserCloudCenHeight--;
00550         }
00551 
00552         while (centerCubeK < 3) {
00553           for (int i = 0; i < laserCloudWidth; i++) {
00554             for (int j = 0; j < laserCloudHeight; j++) {
00555               int k = laserCloudDepth - 1;
00556               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
00557               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00558               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
00559               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00560               for (; k >= 1; k--) {
00561                 laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00562                 laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
00563                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00564                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
00565               }
00566               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00567               laserCloudCubeCornerPointer;
00568               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00569               laserCloudCubeSurfPointer;
00570               laserCloudCubeCornerPointer->clear();
00571               laserCloudCubeSurfPointer->clear();
00572             }
00573           }
00574 
00575           centerCubeK++;
00576           laserCloudCenDepth++;
00577         }
00578       
00579         while (centerCubeK >= laserCloudDepth - 3) {
00580           for (int i = 0; i < laserCloudWidth; i++) {
00581             for (int j = 0; j < laserCloudHeight; j++) {
00582               int k = 0;
00583               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
00584               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00585               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
00586               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
00587               for (; k < laserCloudDepth - 1; k++) {
00588                 laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00589                 laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
00590                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
00591                 laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
00592               }
00593               laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00594               laserCloudCubeCornerPointer;
00595               laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
00596               laserCloudCubeSurfPointer;
00597               laserCloudCubeCornerPointer->clear();
00598               laserCloudCubeSurfPointer->clear();
00599             }
00600           }
00601 
00602           centerCubeK--;
00603           laserCloudCenDepth--;
00604         }
00605 
00606         int laserCloudValidNum = 0;
00607         int laserCloudSurroundNum = 0;
00608         for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
00609           for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
00610             for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
00611               if (i >= 0 && i < laserCloudWidth && 
00612                   j >= 0 && j < laserCloudHeight && 
00613                   k >= 0 && k < laserCloudDepth) {
00614 
00615                 float centerX = 50.0 * (i - laserCloudCenWidth);
00616                 float centerY = 50.0 * (j - laserCloudCenHeight);
00617                 float centerZ = 50.0 * (k - laserCloudCenDepth);
00618 
00619                 bool isInLaserFOV = false;
00620                 for (int ii = -1; ii <= 1; ii += 2) {
00621                   for (int jj = -1; jj <= 1; jj += 2) {
00622                     for (int kk = -1; kk <= 1; kk += 2) {
00623                       float cornerX = centerX + 25.0 * ii;
00624                       float cornerY = centerY + 25.0 * jj;
00625                       float cornerZ = centerZ + 25.0 * kk;
00626 
00627                       float squaredSide1 = (transformTobeMapped[3] - cornerX) 
00628                                          * (transformTobeMapped[3] - cornerX) 
00629                                          + (transformTobeMapped[4] - cornerY) 
00630                                          * (transformTobeMapped[4] - cornerY)
00631                                          + (transformTobeMapped[5] - cornerZ) 
00632                                          * (transformTobeMapped[5] - cornerZ);
00633 
00634                       float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) 
00635                                          + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
00636                                          + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);
00637 
00638                       float check1 = 100.0 + squaredSide1 - squaredSide2
00639                                    - 10.0 * sqrt(3.0) * sqrt(squaredSide1);
00640 
00641                       float check2 = 100.0 + squaredSide1 - squaredSide2
00642                                    + 10.0 * sqrt(3.0) * sqrt(squaredSide1);
00643 
00644                       if (check1 < 0 && check2 > 0) {
00645                         isInLaserFOV = true;
00646                       }
00647                     }
00648                   }
00649                 }
00650 
00651                 if (isInLaserFOV) {
00652                   laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j 
00653                                                        + laserCloudWidth * laserCloudHeight * k;
00654                   laserCloudValidNum++;
00655                 }
00656                 laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j 
00657                                                              + laserCloudWidth * laserCloudHeight * k;
00658                 laserCloudSurroundNum++;
00659               }
00660             }
00661           }
00662         }
00663 
00664         laserCloudCornerFromMap->clear();
00665         laserCloudSurfFromMap->clear();
00666         for (int i = 0; i < laserCloudValidNum; i++) {
00667           *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
00668           *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
00669         }
00670         int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
00671         int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();
00672 
00673         int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
00674         for (int i = 0; i < laserCloudCornerStackNum2; i++) {
00675           pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
00676         }
00677 
00678         int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
00679         for (int i = 0; i < laserCloudSurfStackNum2; i++) {
00680           pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
00681         }
00682 
00683         laserCloudCornerStack->clear();
00684         downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
00685         downSizeFilterCorner.filter(*laserCloudCornerStack);
00686         int laserCloudCornerStackNum = laserCloudCornerStack->points.size();
00687 
00688         laserCloudSurfStack->clear();
00689         downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
00690         downSizeFilterSurf.filter(*laserCloudSurfStack);
00691         int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
00692 
00693         laserCloudCornerStack2->clear();
00694         laserCloudSurfStack2->clear();
00695 
00696         if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
00697           kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
00698           kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
00699 
00700           for (int iterCount = 0; iterCount < 10; iterCount++) {
00701             laserCloudOri->clear();
00702             //laserCloudSel->clear();
00703             //laserCloudCorr->clear();
00704             //laserCloudProj->clear();
00705             coeffSel->clear();
00706 
00707             for (int i = 0; i < laserCloudCornerStackNum; i++) {
00708               pointOri = laserCloudCornerStack->points[i];
00709               pointAssociateToMap(&pointOri, &pointSel);
00710               kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
00711               
00712               if (pointSearchSqDis[4] < 1.0) {
00713                 float cx = 0;
00714                 float cy = 0; 
00715                 float cz = 0;
00716                 for (int j = 0; j < 5; j++) {
00717                   cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
00718                   cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
00719                   cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
00720                 }
00721                 cx /= 5;
00722                 cy /= 5; 
00723                 cz /= 5;
00724 
00725                 float a11 = 0;
00726                 float a12 = 0; 
00727                 float a13 = 0;
00728                 float a22 = 0;
00729                 float a23 = 0; 
00730                 float a33 = 0;
00731                 for (int j = 0; j < 5; j++) {
00732                   float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
00733                   float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
00734                   float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;
00735 
00736                   a11 += ax * ax;
00737                   a12 += ax * ay;
00738                   a13 += ax * az;
00739                   a22 += ay * ay;
00740                   a23 += ay * az;
00741                   a33 += az * az;
00742                 }
00743                 a11 /= 5;
00744                 a12 /= 5; 
00745                 a13 /= 5;
00746                 a22 /= 5;
00747                 a23 /= 5; 
00748                 a33 /= 5;
00749 
00750                 matA1.at<float>(0, 0) = a11;
00751                 matA1.at<float>(0, 1) = a12;
00752                 matA1.at<float>(0, 2) = a13;
00753                 matA1.at<float>(1, 0) = a12;
00754                 matA1.at<float>(1, 1) = a22;
00755                 matA1.at<float>(1, 2) = a23;
00756                 matA1.at<float>(2, 0) = a13;
00757                 matA1.at<float>(2, 1) = a23;
00758                 matA1.at<float>(2, 2) = a33;
00759 
00760                 cv::eigen(matA1, matD1, matV1);
00761 
00762                 if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
00763 
00764                   float x0 = pointSel.x;
00765                   float y0 = pointSel.y;
00766                   float z0 = pointSel.z;
00767                   float x1 = cx + 0.1 * matV1.at<float>(0, 0);
00768                   float y1 = cy + 0.1 * matV1.at<float>(0, 1);
00769                   float z1 = cz + 0.1 * matV1.at<float>(0, 2);
00770                   float x2 = cx - 0.1 * matV1.at<float>(0, 0);
00771                   float y2 = cy - 0.1 * matV1.at<float>(0, 1);
00772                   float z2 = cz - 0.1 * matV1.at<float>(0, 2);
00773 
00774                   float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
00775                              * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
00776                              + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
00777                              * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
00778                              + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
00779                              * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
00780 
00781                   float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
00782 
00783                   float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
00784                            + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;
00785 
00786                   float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
00787                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
00788 
00789                   float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
00790                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
00791 
00792                   float ld2 = a012 / l12;
00793 
00794                   pointProj = pointSel;
00795                   pointProj.x -= la * ld2;
00796                   pointProj.y -= lb * ld2;
00797                   pointProj.z -= lc * ld2;
00798 
00799                   float s = 1 - 0.9 * fabs(ld2);
00800 
00801                   coeff.x = s * la;
00802                   coeff.y = s * lb;
00803                   coeff.z = s * lc;
00804                   coeff.intensity = s * ld2;
00805 
00806                   if (s > 0.1) {
00807                     laserCloudOri->push_back(pointOri);
00808                     //laserCloudSel->push_back(pointSel);
00809                     //laserCloudProj->push_back(pointProj);
00810                     //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
00811                     //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
00812                     //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
00813                     //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
00814                     //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
00815                     coeffSel->push_back(coeff);
00816                   }
00817                 }
00818               }
00819             }
00820 
00821             for (int i = 0; i < laserCloudSurfStackNum; i++) {
00822               pointOri = laserCloudSurfStack->points[i];
00823               pointAssociateToMap(&pointOri, &pointSel); 
00824               kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
00825 
00826               if (pointSearchSqDis[4] < 1.0) {
00827                 for (int j = 0; j < 5; j++) {
00828                   matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
00829                   matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
00830                   matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
00831                 }
00832                 cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);
00833 
00834                 float pa = matX0.at<float>(0, 0);
00835                 float pb = matX0.at<float>(1, 0);
00836                 float pc = matX0.at<float>(2, 0);
00837                 float pd = 1;
00838  
00839                 float ps = sqrt(pa * pa + pb * pb + pc * pc);
00840                 pa /= ps;
00841                 pb /= ps;
00842                 pc /= ps;
00843                 pd /= ps;
00844 
00845                 bool planeValid = true;
00846                 for (int j = 0; j < 5; j++) {
00847                   if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
00848                       pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
00849                       pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
00850                     planeValid = false;
00851                     break;
00852                   }
00853                 }
00854 
00855                 if (planeValid) {
00856                   float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
00857 
00858                   pointProj = pointSel;
00859                   pointProj.x -= pa * pd2;
00860                   pointProj.y -= pb * pd2;
00861                   pointProj.z -= pc * pd2;
00862 
00863                   float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
00864                           + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
00865 
00866                   coeff.x = s * pa;
00867                   coeff.y = s * pb;
00868                   coeff.z = s * pc;
00869                   coeff.intensity = s * pd2;
00870 
00871                   if (s > 0.1) {
00872                     laserCloudOri->push_back(pointOri);
00873                     //laserCloudSel->push_back(pointSel);
00874                     //laserCloudProj->push_back(pointProj);
00875                     //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
00876                     //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
00877                     //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
00878                     //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
00879                     //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
00880                     coeffSel->push_back(coeff);
00881                   }
00882                 }
00883               }
00884             }
00885 
00886             float srx = sin(transformTobeMapped[0]);
00887             float crx = cos(transformTobeMapped[0]);
00888             float sry = sin(transformTobeMapped[1]);
00889             float cry = cos(transformTobeMapped[1]);
00890             float srz = sin(transformTobeMapped[2]);
00891             float crz = cos(transformTobeMapped[2]);
00892 
00893             int laserCloudSelNum = laserCloudOri->points.size();
00894             if (laserCloudSelNum < 50) {
00895               continue;
00896             }
00897 
00898             cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
00899             cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
00900             cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
00901             cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
00902             cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
00903             cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
00904             for (int i = 0; i < laserCloudSelNum; i++) {
00905               pointOri = laserCloudOri->points[i];
00906               coeff = coeffSel->points[i];
00907 
00908               float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
00909                         + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
00910                         + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;
00911 
00912               float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
00913                         + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
00914                         + ((-cry*crz - srx*sry*srz)*pointOri.x 
00915                         + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;
00916 
00917               float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
00918                         + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
00919                         + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
00920 
00921               matA.at<float>(i, 0) = arx;
00922               matA.at<float>(i, 1) = ary;
00923               matA.at<float>(i, 2) = arz;
00924               matA.at<float>(i, 3) = coeff.x;
00925               matA.at<float>(i, 4) = coeff.y;
00926               matA.at<float>(i, 5) = coeff.z;
00927               matB.at<float>(i, 0) = -coeff.intensity;
00928             }
00929             cv::transpose(matA, matAt);
00930             matAtA = matAt * matA;
00931             matAtB = matAt * matB;
00932             cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
00933 
00934             if (iterCount == 0) {
00935               cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
00936               cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
00937               cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
00938 
00939               cv::eigen(matAtA, matE, matV);
00940               matV.copyTo(matV2);
00941 
00942               isDegenerate = false;
00943               float eignThre[6] = {100, 100, 100, 100, 100, 100};
00944               for (int i = 5; i >= 0; i--) {
00945                 if (matE.at<float>(0, i) < eignThre[i]) {
00946                   for (int j = 0; j < 6; j++) {
00947                     matV2.at<float>(i, j) = 0;
00948                   }
00949                   isDegenerate = true;
00950                 } else {
00951                   break;
00952                 }
00953               }
00954               matP = matV.inv() * matV2;
00955             }
00956 
00957             if (isDegenerate /*&& 0*/) {
00958               cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
00959               matX.copyTo(matX2);
00960               matX = matP * matX2;
00961 
00962               //ROS_INFO ("laser mapping degenerate");
00963             }
00964 
00965             transformTobeMapped[0] += matX.at<float>(0, 0);
00966             transformTobeMapped[1] += matX.at<float>(1, 0);
00967             transformTobeMapped[2] += matX.at<float>(2, 0);
00968             transformTobeMapped[3] += matX.at<float>(3, 0);
00969             transformTobeMapped[4] += matX.at<float>(4, 0);
00970             transformTobeMapped[5] += matX.at<float>(5, 0);
00971 
00972             float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
00973                          + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
00974                          + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
00975             float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
00976                          + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
00977                          + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);
00978 
00979             if (deltaR < 0.05 && deltaT < 0.05) {
00980               break;
00981             }
00982 
00983             //ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
00984           }
00985 
00986           transformUpdate();
00987         }
00988 
00989         for (int i = 0; i < laserCloudCornerStackNum; i++) {
00990           pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
00991 
00992           int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
00993           int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
00994           int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
00995 
00996           if (pointSel.x + 25.0 < 0) cubeI--;
00997           if (pointSel.y + 25.0 < 0) cubeJ--;
00998           if (pointSel.z + 25.0 < 0) cubeK--;
00999 
01000           if (cubeI >= 0 && cubeI < laserCloudWidth && 
01001               cubeJ >= 0 && cubeJ < laserCloudHeight && 
01002               cubeK >= 0 && cubeK < laserCloudDepth) {
01003             int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
01004             laserCloudCornerArray[cubeInd]->push_back(pointSel);
01005           }
01006         }
01007 
01008         for (int i = 0; i < laserCloudSurfStackNum; i++) {
01009           pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
01010 
01011           int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
01012           int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
01013           int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
01014 
01015           if (pointSel.x + 25.0 < 0) cubeI--;
01016           if (pointSel.y + 25.0 < 0) cubeJ--;
01017           if (pointSel.z + 25.0 < 0) cubeK--;
01018 
01019           if (cubeI >= 0 && cubeI < laserCloudWidth && 
01020               cubeJ >= 0 && cubeJ < laserCloudHeight && 
01021               cubeK >= 0 && cubeK < laserCloudDepth) {
01022             int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
01023             laserCloudSurfArray[cubeInd]->push_back(pointSel);
01024           }
01025         }
01026 
01027         for (int i = 0; i < laserCloudValidNum; i++) {
01028           int ind = laserCloudValidInd[i];
01029 
01030           laserCloudCornerArray2[ind]->clear();
01031           downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
01032           downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);
01033 
01034           laserCloudSurfArray2[ind]->clear();
01035           downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
01036           downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);
01037 
01038           pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
01039           laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
01040           laserCloudCornerArray2[ind] = laserCloudTemp;
01041 
01042           laserCloudTemp = laserCloudSurfArray[ind];
01043           laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
01044           laserCloudSurfArray2[ind] = laserCloudTemp;
01045         }
01046 
01047         mapFrameCount++;
01048         if (mapFrameCount >= mapFrameNum) {
01049           mapFrameCount = 0;
01050 
01051           laserCloudSurround2->clear();
01052           for (int i = 0; i < laserCloudSurroundNum; i++) {
01053             int ind = laserCloudSurroundInd[i];
01054             *laserCloudSurround2 += *laserCloudCornerArray[ind];
01055             *laserCloudSurround2 += *laserCloudSurfArray[ind];
01056           }
01057 
01058           laserCloudSurround->clear();
01059           downSizeFilterCorner.setInputCloud(laserCloudSurround2);
01060           downSizeFilterCorner.filter(*laserCloudSurround);
01061 
01062           sensor_msgs::PointCloud2 laserCloudSurround3;
01063           pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
01064           laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01065           laserCloudSurround3.header.frame_id = "/camera_init";
01066           pubLaserCloudSurround.publish(laserCloudSurround3);
01067         }
01068 
01069         int laserCloudFullResNum = laserCloudFullRes->points.size();
01070         for (int i = 0; i < laserCloudFullResNum; i++) {
01071           pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
01072         }
01073 
01074         sensor_msgs::PointCloud2 laserCloudFullRes3;
01075         pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
01076         laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01077         laserCloudFullRes3.header.frame_id = "/camera_init";
01078         pubLaserCloudFullRes.publish(laserCloudFullRes3);
01079 
01080         geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
01081                                   (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);
01082 
01083         odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01084         odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
01085         odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
01086         odomAftMapped.pose.pose.orientation.z = geoQuat.x;
01087         odomAftMapped.pose.pose.orientation.w = geoQuat.w;
01088         odomAftMapped.pose.pose.position.x = transformAftMapped[3];
01089         odomAftMapped.pose.pose.position.y = transformAftMapped[4];
01090         odomAftMapped.pose.pose.position.z = transformAftMapped[5];
01091         odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
01092         odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
01093         odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
01094         odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
01095         odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
01096         odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
01097         pubOdomAftMapped.publish(odomAftMapped);
01098 
01099         aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
01100         aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
01101         aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], 
01102                                              transformAftMapped[4], transformAftMapped[5]));
01103         tfBroadcaster.sendTransform(aftMappedTrans);
01104 
01105         /*sensor_msgs::PointCloud2 pc12;
01106         pcl::toROSMsg(*laserCloudCornerStack, pc12);
01107         pc12.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01108         pc12.header.frame_id = "/camera";
01109         pub1.publish(pc12);
01110 
01111         sensor_msgs::PointCloud2 pc22;
01112         pcl::toROSMsg(*laserCloudSurfStack, pc22);
01113         pc22.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01114         pc22.header.frame_id = "/camera";
01115         pub2.publish(pc22);
01116 
01117         sensor_msgs::PointCloud2 pc32;
01118         pcl::toROSMsg(*laserCloudSel, pc32);
01119         pc32.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01120         pc32.header.frame_id = "/camera";
01121         pub3.publish(pc32);
01122 
01123         sensor_msgs::PointCloud2 pc42;
01124         pcl::toROSMsg(*laserCloudProj, pc42);
01125         pc42.header.stamp = ros::Time().fromSec(timeLaserOdometry);
01126         pc42.header.frame_id = "/camera";
01127         pub4.publish(pc42);*/
01128       }
01129     }
01130 
01131     status = ros::ok();
01132     rate.sleep();
01133   }
01134 
01135   return 0;
01136 }
