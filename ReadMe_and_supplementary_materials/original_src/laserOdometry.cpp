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
00027 const int skipFrameNum = 1;
00028 bool systemInited = false;
00029 
00030 double timeCornerPointsSharp = 0;
00031 double timeCornerPointsLessSharp = 0;
00032 double timeSurfPointsFlat = 0;
00033 double timeSurfPointsLessFlat = 0;
00034 double timeLaserCloudFullRes = 0;
00035 double timeImuTrans = 0;
00036 
00037 bool newCornerPointsSharp = false;
00038 bool newCornerPointsLessSharp = false;
00039 bool newSurfPointsFlat = false;
00040 bool newSurfPointsLessFlat = false;
00041 bool newLaserCloudFullRes = false;
00042 bool newImuTrans = false;
00043 
00044 pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
00045 pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());
00046 pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
00047 pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());
00048 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
00049 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
00050 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());
00051 //pcl::PointCloud<pcl::PointXYZI>::Ptr pointSearchCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
00052 //pcl::PointCloud<pcl::PointXYZI>::Ptr pointSearchSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
00053 //pcl::PointCloud<pcl::PointXYZI>::Ptr pointProjCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
00054 //pcl::PointCloud<pcl::PointXYZI>::Ptr pointProjSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
00055 pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
00056 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>());
00057 pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());
00058 pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
00059 pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
00060 
00061 int laserCloudCornerLastNum;
00062 int laserCloudSurfLastNum;
00063 
00064 int pointSelCornerInd[40000];
00065 float pointSearchCornerInd1[40000];
00066 float pointSearchCornerInd2[40000];
00067 
00068 int pointSelSurfInd[40000];
00069 float pointSearchSurfInd1[40000];
00070 float pointSearchSurfInd2[40000];
00071 float pointSearchSurfInd3[40000];
00072 
00073 float transform[6] = {0};
00074 float transformSum[6] = {0};
00075 
00076 float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
00077 float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
00078 float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
00079 float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;
00080 
00081 void TransformToStart(pcl::PointXYZI *pi, pcl::PointXYZI *po)
00082 {
00083   float s = 10 * (pi->intensity - int(pi->intensity));
00084 
00085   float rx = s * transform[0];
00086   float ry = s * transform[1];
00087   float rz = s * transform[2];
00088   float tx = s * transform[3];
00089   float ty = s * transform[4];
00090   float tz = s * transform[5];
00091 
00092   float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
00093   float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
00094   float z1 = (pi->z - tz);
00095 
00096   float x2 = x1;
00097   float y2 = cos(rx) * y1 + sin(rx) * z1;
00098   float z2 = -sin(rx) * y1 + cos(rx) * z1;
00099 
00100   po->x = cos(ry) * x2 - sin(ry) * z2;
00101   po->y = y2;
00102   po->z = sin(ry) * x2 + cos(ry) * z2;
00103   po->intensity = pi->intensity;
00104 }
00105 
00106 void TransformToEnd(pcl::PointXYZI *pi, pcl::PointXYZI *po)
00107 {
00108   float s = 10 * (pi->intensity - int(pi->intensity));
00109 
00110   float rx = s * transform[0];
00111   float ry = s * transform[1];
00112   float rz = s * transform[2];
00113   float tx = s * transform[3];
00114   float ty = s * transform[4];
00115   float tz = s * transform[5];
00116 
00117   float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
00118   float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
00119   float z1 = (pi->z - tz);
00120 
00121   float x2 = x1;
00122   float y2 = cos(rx) * y1 + sin(rx) * z1;
00123   float z2 = -sin(rx) * y1 + cos(rx) * z1;
00124 
00125   float x3 = cos(ry) * x2 - sin(ry) * z2;
00126   float y3 = y2;
00127   float z3 = sin(ry) * x2 + cos(ry) * z2;
00128 
00129   rx = transform[0];
00130   ry = transform[1];
00131   rz = transform[2];
00132   tx = transform[3];
00133   ty = transform[4];
00134   tz = transform[5];
00135 
00136   float x4 = cos(ry) * x3 + sin(ry) * z3;
00137   float y4 = y3;
00138   float z4 = -sin(ry) * x3 + cos(ry) * z3;
00139 
00140   float x5 = x4;
00141   float y5 = cos(rx) * y4 - sin(rx) * z4;
00142   float z5 = sin(rx) * y4 + cos(rx) * z4;
00143 
00144   float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
00145   float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
00146   float z6 = z5 + tz;
00147 
00148   float x7 = cos(imuRollStart) * (x6 - imuShiftFromStartX) 
00149            - sin(imuRollStart) * (y6 - imuShiftFromStartY);
00150   float y7 = sin(imuRollStart) * (x6 - imuShiftFromStartX) 
00151            + cos(imuRollStart) * (y6 - imuShiftFromStartY);
00152   float z7 = z6 - imuShiftFromStartZ;
00153 
00154   float x8 = x7;
00155   float y8 = cos(imuPitchStart) * y7 - sin(imuPitchStart) * z7;
00156   float z8 = sin(imuPitchStart) * y7 + cos(imuPitchStart) * z7;
00157 
00158   float x9 = cos(imuYawStart) * x8 + sin(imuYawStart) * z8;
00159   float y9 = y8;
00160   float z9 = -sin(imuYawStart) * x8 + cos(imuYawStart) * z8;
00161 
00162   float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
00163   float y10 = y9;
00164   float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;
00165 
00166   float x11 = x10;
00167   float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
00168   float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;
00169 
00170   po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
00171   po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
00172   po->z = z11;
00173   po->intensity = int(pi->intensity);
00174 }
00175 
00176 void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
00177                        float alx, float aly, float alz, float &acx, float &acy, float &acz)
00178 {
00179   float sbcx = sin(bcx);
00180   float cbcx = cos(bcx);
00181   float sbcy = sin(bcy);
00182   float cbcy = cos(bcy);
00183   float sbcz = sin(bcz);
00184   float cbcz = cos(bcz);
00185 
00186   float sblx = sin(blx);
00187   float cblx = cos(blx);
00188   float sbly = sin(bly);
00189   float cbly = cos(bly);
00190   float sblz = sin(blz);
00191   float cblz = cos(blz);
00192 
00193   float salx = sin(alx);
00194   float calx = cos(alx);
00195   float saly = sin(aly);
00196   float caly = cos(aly);
00197   float salz = sin(alz);
00198   float calz = cos(alz);
00199 
00200   float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
00201             - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00202             - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00203             - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00204             - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
00205   acx = -asin(srx);
00206 
00207   float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00208                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00209                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00210                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
00211                + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
00212   float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
00213                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
00214                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
00215                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
00216                + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
00217   acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
00218   
00219   float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
00220                - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
00221                - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
00222                + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
00223                - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
00224                + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
00225                + calx*cblx*salz*sblz);
00226   float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
00227                - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
00228                + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
00229                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
00230                + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
00231                - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
00232                - calx*calz*cblx*sblz);
00233   acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
00234 }
00235 
00236 void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
00237                         float &ox, float &oy, float &oz)
00238 {
00239   float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
00240   ox = -asin(srx);
00241 
00242   float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
00243                + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
00244   float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
00245                - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
00246   oy = atan2(srycrx / cos(ox), crycrx / cos(ox));
00247 
00248   float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
00249                + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
00250   float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
00251                - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
00252   oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
00253 }
00254 
00255 void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharp2)
00256 {
00257   timeCornerPointsSharp = cornerPointsSharp2->header.stamp.toSec();
00258 
00259   cornerPointsSharp->clear();
00260   pcl::fromROSMsg(*cornerPointsSharp2, *cornerPointsSharp);
00261 
00262   newCornerPointsSharp = true;
00263 }
00264 
00265 void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharp2)
00266 {
00267   timeCornerPointsLessSharp = cornerPointsLessSharp2->header.stamp.toSec();
00268 
00269   cornerPointsLessSharp->clear();
00270   pcl::fromROSMsg(*cornerPointsLessSharp2, *cornerPointsLessSharp);
00271 
00272   newCornerPointsLessSharp = true;
00273 }
00274 
00275 void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlat2)
00276 {
00277   timeSurfPointsFlat = surfPointsFlat2->header.stamp.toSec();
00278 
00279   surfPointsFlat->clear();
00280   pcl::fromROSMsg(*surfPointsFlat2, *surfPointsFlat);
00281 
00282   newSurfPointsFlat = true;
00283 }
00284 
00285 void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlat2)
00286 {
00287   timeSurfPointsLessFlat = surfPointsLessFlat2->header.stamp.toSec();
00288 
00289   surfPointsLessFlat->clear();
00290   pcl::fromROSMsg(*surfPointsLessFlat2, *surfPointsLessFlat);
00291 
00292   newSurfPointsLessFlat = true;
00293 }
00294 
00295 void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
00296 {
00297   timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
00298 
00299   laserCloudFullRes->clear();
00300   pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
00301 
00302   newLaserCloudFullRes = true;
00303 }
00304 
00305 void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2)
00306 {
00307   timeImuTrans = imuTrans2->header.stamp.toSec();
00308 
00309   imuTrans->clear();
00310   pcl::fromROSMsg(*imuTrans2, *imuTrans);
00311 
00312   imuPitchStart = imuTrans->points[0].x;
00313   imuYawStart = imuTrans->points[0].y;
00314   imuRollStart = imuTrans->points[0].z;
00315 
00316   imuPitchLast = imuTrans->points[1].x;
00317   imuYawLast = imuTrans->points[1].y;
00318   imuRollLast = imuTrans->points[1].z;
00319 
00320   imuShiftFromStartX = imuTrans->points[2].x;
00321   imuShiftFromStartY = imuTrans->points[2].y;
00322   imuShiftFromStartZ = imuTrans->points[2].z;
00323 
00324   imuVeloFromStartX = imuTrans->points[3].x;
00325   imuVeloFromStartY = imuTrans->points[3].y;
00326   imuVeloFromStartZ = imuTrans->points[3].z;
00327 
00328   newImuTrans = true;
00329 }
00330 
00331 int main(int argc, char** argv)
00332 {
00333   ros::init(argc, argv, "laserOdometry");
00334   ros::NodeHandle nh;
00335 
00336   ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>
00337                                          ("/laser_cloud_sharp", 2, laserCloudSharpHandler);
00338 
00339   ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>
00340                                              ("/laser_cloud_less_sharp", 2, laserCloudLessSharpHandler);
00341 
00342   ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>
00343                                       ("/laser_cloud_flat", 2, laserCloudFlatHandler);
00344 
00345   ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>
00346                                           ("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);
00347 
00348   ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> 
00349                                          ("/velodyne_cloud_2", 2, laserCloudFullResHandler);
00350 
00351   ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2> 
00352                                 ("/imu_trans", 5, imuTransHandler);
00353 
00354   ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>
00355                                            ("/laser_cloud_corner_last", 2);
00356 
00357   ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>
00358                                          ("/laser_cloud_surf_last", 2);
00359 
00360   ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> 
00361                                         ("/velodyne_cloud_3", 2);
00362 
00363   //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 2);
00364 
00365   //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 2);
00366 
00367   //ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 2);
00368 
00369   //ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 2);
00370 
00371   ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
00372   nav_msgs::Odometry laserOdometry;
00373   laserOdometry.header.frame_id = "/camera_init";
00374   laserOdometry.child_frame_id = "/laser_odom";
00375 
00376   tf::TransformBroadcaster tfBroadcaster;
00377   tf::StampedTransform laserOdometryTrans;
00378   laserOdometryTrans.frame_id_ = "/camera_init";
00379   laserOdometryTrans.child_frame_id_ = "/laser_odom";
00380 
00381   std::vector<int> pointSearchInd;
00382   std::vector<float> pointSearchSqDis;
00383 
00384   pcl::PointXYZI pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;
00385 
00386   bool isDegenerate = false;
00387   cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
00388 
00389   int frameCount = skipFrameNum;
00390   ros::Rate rate(100);
00391   bool status = ros::ok();
00392   while (status) {
00393     ros::spinOnce();
00394 
00395     if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat && 
00396         newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans &&
00397         fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
00398         fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
00399         fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
00400         fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005 &&
00401         fabs(timeImuTrans - timeSurfPointsLessFlat) < 0.005) {
00402       newCornerPointsSharp = false;
00403       newCornerPointsLessSharp = false;
00404       newSurfPointsFlat = false;
00405       newSurfPointsLessFlat = false;
00406       newLaserCloudFullRes = false;
00407       newImuTrans = false;
00408 
00409       if (!systemInited) {
00410         pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
00411         cornerPointsLessSharp = laserCloudCornerLast;
00412         laserCloudCornerLast = laserCloudTemp;
00413 
00414         laserCloudTemp = surfPointsLessFlat;
00415         surfPointsLessFlat = laserCloudSurfLast;
00416         laserCloudSurfLast = laserCloudTemp;
00417 
00418         kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
00419         kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
00420 
00421         sensor_msgs::PointCloud2 laserCloudCornerLast2;
00422         pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
00423         laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00424         laserCloudCornerLast2.header.frame_id = "/camera";
00425         pubLaserCloudCornerLast.publish(laserCloudCornerLast2);
00426 
00427         sensor_msgs::PointCloud2 laserCloudSurfLast2;
00428         pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
00429         laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00430         laserCloudSurfLast2.header.frame_id = "/camera";
00431         pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
00432 
00433         transformSum[0] += imuPitchStart;
00434         //transformSum[1] += imuYawStart;
00435         transformSum[2] += imuRollStart;
00436 
00437         systemInited = true;
00438         continue;
00439       }
00440 
00441       laserCloudOri->clear();
00442       //pointSearchCornerLast->clear();
00443       //pointProjCornerLast->clear();
00444       //pointSearchSurfLast->clear();
00445       //pointProjSurfLast->clear();
00446       coeffSel->clear();
00447 
00448       transform[3] -= imuVeloFromStartX * scanPeriod;
00449       transform[4] -= imuVeloFromStartY * scanPeriod;
00450       transform[5] -= imuVeloFromStartZ * scanPeriod;
00451 
00452       if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
00453         int cornerPointsSharpNum = cornerPointsSharp->points.size();
00454         int surfPointsFlatNum = surfPointsFlat->points.size();
00455         for (int iterCount = 0; iterCount < 25; iterCount++) {
00456           for (int i = 0; i < cornerPointsSharpNum; i++) {
00457             TransformToStart(&cornerPointsSharp->points[i], &pointSel);
00458 
00459             if (iterCount % 5 == 0) {
00460               kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
00461 
00462               int closestPointInd = -1, minPointInd2 = -1;
00463               if (pointSearchSqDis[0] < 25) {
00464                 closestPointInd = pointSearchInd[0];
00465                 int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);
00466 
00467                 float pointSqDis, minPointSqDis2 = 25;
00468                 for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
00469                   if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
00470                     break;
00471                   }
00472 
00473                   pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
00474                                (laserCloudCornerLast->points[j].x - pointSel.x) + 
00475                                (laserCloudCornerLast->points[j].y - pointSel.y) * 
00476                                (laserCloudCornerLast->points[j].y - pointSel.y) + 
00477                                (laserCloudCornerLast->points[j].z - pointSel.z) * 
00478                                (laserCloudCornerLast->points[j].z - pointSel.z);
00479 
00480                   if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
00481                     if (pointSqDis < minPointSqDis2) {
00482                       minPointSqDis2 = pointSqDis;
00483                       minPointInd2 = j;
00484                     }
00485                   }
00486                 }
00487                 for (int j = closestPointInd - 1; j >= 0; j--) {
00488                   if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
00489                     break;
00490                   }
00491 
00492                   pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
00493                                (laserCloudCornerLast->points[j].x - pointSel.x) + 
00494                                (laserCloudCornerLast->points[j].y - pointSel.y) * 
00495                                (laserCloudCornerLast->points[j].y - pointSel.y) + 
00496                                (laserCloudCornerLast->points[j].z - pointSel.z) * 
00497                                (laserCloudCornerLast->points[j].z - pointSel.z);
00498 
00499                   if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
00500                     if (pointSqDis < minPointSqDis2) {
00501                       minPointSqDis2 = pointSqDis;
00502                       minPointInd2 = j;
00503                     }
00504                   }
00505                 }
00506               }
00507 
00508               pointSearchCornerInd1[i] = closestPointInd;
00509               pointSearchCornerInd2[i] = minPointInd2;
00510             }
00511 
00512             if (pointSearchCornerInd2[i] >= 0) {
00513               tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
00514               tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];
00515 
00516               float x0 = pointSel.x;
00517               float y0 = pointSel.y;
00518               float z0 = pointSel.z;
00519               float x1 = tripod1.x;
00520               float y1 = tripod1.y;
00521               float z1 = tripod1.z;
00522               float x2 = tripod2.x;
00523               float y2 = tripod2.y;
00524               float z2 = tripod2.z;
00525 
00526               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
00527                          * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
00528                          + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
00529                          * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
00530                          + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
00531                          * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
00532 
00533               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
00534 
00535               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
00536                        + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;
00537 
00538               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
00539                        - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
00540 
00541               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
00542                        + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;
00543 
00544               float ld2 = a012 / l12;
00545 
00546               pointProj = pointSel;
00547               pointProj.x -= la * ld2;
00548               pointProj.y -= lb * ld2;
00549               pointProj.z -= lc * ld2;
00550 
00551               float s = 1;
00552               if (iterCount >= 5) {
00553                 s = 1 - 1.8 * fabs(ld2);
00554               }
00555 
00556               coeff.x = s * la;
00557               coeff.y = s * lb;
00558               coeff.z = s * lc;
00559               coeff.intensity = s * ld2;
00560 
00561               if (s > 0.1 && ld2 != 0) {
00562                 laserCloudOri->push_back(cornerPointsSharp->points[i]);
00563                 //pointSearchCornerLast->push_back(tripod1);
00564                 //pointSearchCornerLast->push_back(tripod2);
00565                 //pointProjCornerLast->push_back(pointProj);
00566                 coeffSel->push_back(coeff);
00567               }
00568             }
00569           }
00570 
00571           for (int i = 0; i < surfPointsFlatNum; i++) {
00572             TransformToStart(&surfPointsFlat->points[i], &pointSel);
00573 
00574             if (iterCount % 5 == 0) {
00575               kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
00576 
00577               int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
00578               if (pointSearchSqDis[0] < 25) {
00579                 closestPointInd = pointSearchInd[0];
00580                 int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);
00581 
00582                 float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
00583                 for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
00584                   if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
00585                     break;
00586                   }
00587 
00588                   pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
00589                                (laserCloudSurfLast->points[j].x - pointSel.x) + 
00590                                (laserCloudSurfLast->points[j].y - pointSel.y) * 
00591                                (laserCloudSurfLast->points[j].y - pointSel.y) + 
00592                                (laserCloudSurfLast->points[j].z - pointSel.z) * 
00593                                (laserCloudSurfLast->points[j].z - pointSel.z);
00594 
00595                   if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
00596                      if (pointSqDis < minPointSqDis2) {
00597                        minPointSqDis2 = pointSqDis;
00598                        minPointInd2 = j;
00599                      }
00600                   } else {
00601                      if (pointSqDis < minPointSqDis3) {
00602                        minPointSqDis3 = pointSqDis;
00603                        minPointInd3 = j;
00604                      }
00605                   }
00606                 }
00607                 for (int j = closestPointInd - 1; j >= 0; j--) {
00608                   if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
00609                     break;
00610                   }
00611 
00612                   pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
00613                                (laserCloudSurfLast->points[j].x - pointSel.x) + 
00614                                (laserCloudSurfLast->points[j].y - pointSel.y) * 
00615                                (laserCloudSurfLast->points[j].y - pointSel.y) + 
00616                                (laserCloudSurfLast->points[j].z - pointSel.z) * 
00617                                (laserCloudSurfLast->points[j].z - pointSel.z);
00618 
00619                   if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
00620                     if (pointSqDis < minPointSqDis2) {
00621                       minPointSqDis2 = pointSqDis;
00622                       minPointInd2 = j;
00623                     }
00624                   } else {
00625                     if (pointSqDis < minPointSqDis3) {
00626                       minPointSqDis3 = pointSqDis;
00627                       minPointInd3 = j;
00628                     }
00629                   }
00630                 }
00631               }
00632 
00633               pointSearchSurfInd1[i] = closestPointInd;
00634               pointSearchSurfInd2[i] = minPointInd2;
00635               pointSearchSurfInd3[i] = minPointInd3;
00636             }
00637 
00638             if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
00639               tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
00640               tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
00641               tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];
00642 
00643               float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
00644                        - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
00645               float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
00646                        - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
00647               float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
00648                        - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
00649               float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);
00650 
00651               float ps = sqrt(pa * pa + pb * pb + pc * pc);
00652               pa /= ps;
00653               pb /= ps;
00654               pc /= ps;
00655               pd /= ps;
00656 
00657               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
00658 
00659               pointProj = pointSel;
00660               pointProj.x -= pa * pd2;
00661               pointProj.y -= pb * pd2;
00662               pointProj.z -= pc * pd2;
00663 
00664               float s = 1;
00665               if (iterCount >= 5) {
00666                 s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
00667                   + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
00668               }
00669 
00670               coeff.x = s * pa;
00671               coeff.y = s * pb;
00672               coeff.z = s * pc;
00673               coeff.intensity = s * pd2;
00674 
00675               if (s > 0.1 && pd2 != 0) {
00676                 laserCloudOri->push_back(surfPointsFlat->points[i]);
00677                 //pointSearchSurfLast->push_back(tripod1);
00678                 //pointSearchSurfLast->push_back(tripod2);
00679                 //pointSearchSurfLast->push_back(tripod3);
00680                 //pointProjSurfLast->push_back(pointProj);
00681                 coeffSel->push_back(coeff);
00682               }
00683             }
00684           }
00685 
00686           int pointSelNum = laserCloudOri->points.size();
00687           if (pointSelNum < 10) {
00688             continue;
00689           }
00690 
00691           cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
00692           cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
00693           cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
00694           cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
00695           cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
00696           cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
00697           for (int i = 0; i < pointSelNum; i++) {
00698             pointOri = laserCloudOri->points[i];
00699             coeff = coeffSel->points[i];
00700 
00701             float s = 1; //10 * (pointOri.intensity - int(pointOri.intensity));
00702 
00703             float srx = sin(s * transform[0]);
00704             float crx = cos(s * transform[0]);
00705             float sry = sin(s * transform[1]);
00706             float cry = cos(s * transform[1]);
00707             float srz = sin(s * transform[2]);
00708             float crz = cos(s * transform[2]);
00709             float tx = s * transform[3];
00710             float ty = s * transform[4];
00711             float tz = s * transform[5];
00712 
00713             float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z 
00714                       + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
00715                       + (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
00716                       + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
00717                       + (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
00718                       + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;
00719 
00720             float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x 
00721                       + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z 
00722                       + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx) 
00723                       + s*tz*crx*cry) * coeff.x
00724                       + ((s*cry*crz - s*srx*sry*srz)*pointOri.x 
00725                       + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
00726                       + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry) 
00727                       - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;
00728 
00729             float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
00730                       + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
00731                       + (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
00732                       + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
00733                       + ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
00734                       + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;
00735 
00736             float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y 
00737                       - s*(crz*sry + cry*srx*srz) * coeff.z;
00738   
00739             float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y 
00740                       - s*(sry*srz - cry*crz*srx) * coeff.z;
00741   
00742             float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;
00743 
00744             float d2 = coeff.intensity;
00745 
00746             matA.at<float>(i, 0) = arx;
00747             matA.at<float>(i, 1) = ary;
00748             matA.at<float>(i, 2) = arz;
00749             matA.at<float>(i, 3) = atx;
00750             matA.at<float>(i, 4) = aty;
00751             matA.at<float>(i, 5) = atz;
00752             matB.at<float>(i, 0) = -0.05 * d2;
00753           }
00754           cv::transpose(matA, matAt);
00755           matAtA = matAt * matA;
00756           matAtB = matAt * matB;
00757           cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
00758 
00759           if (iterCount == 0) {
00760             cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
00761             cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
00762             cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
00763 
00764             cv::eigen(matAtA, matE, matV);
00765             matV.copyTo(matV2);
00766 
00767             isDegenerate = false;
00768             float eignThre[6] = {10, 10, 10, 10, 10, 10};
00769             for (int i = 5; i >= 0; i--) {
00770               if (matE.at<float>(0, i) < eignThre[i]) {
00771                 for (int j = 0; j < 6; j++) {
00772                   matV2.at<float>(i, j) = 0;
00773                 }
00774                 isDegenerate = true;
00775               } else {
00776                 break;
00777               }
00778             }
00779             matP = matV.inv() * matV2;
00780           }
00781 
00782           if (isDegenerate) {
00783             cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
00784             matX.copyTo(matX2);
00785             matX = matP * matX2;
00786 
00787             //ROS_INFO ("laser odometry degenerate");
00788           }
00789 
00790           transform[0] += matX.at<float>(0, 0);
00791           transform[1] += matX.at<float>(1, 0);
00792           transform[2] += matX.at<float>(2, 0);
00793           transform[3] += matX.at<float>(3, 0);
00794           transform[4] += matX.at<float>(4, 0);
00795           transform[5] += matX.at<float>(5, 0);
00796 
00797           float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
00798                        + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
00799                        + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
00800           float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
00801                        + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
00802                        + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);
00803 
00804           if (deltaR < 0.1 && deltaT < 0.1) {
00805             break;
00806           }
00807 
00808           //ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
00809         }
00810       }
00811 
00812       float rx, ry, rz, tx, ty, tz;
00813       AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
00814                          -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);
00815 
00816       float x1 = cos(rz) * (transform[3] - imuShiftFromStartX) 
00817                - sin(rz) * (transform[4] - imuShiftFromStartY);
00818       float y1 = sin(rz) * (transform[3] - imuShiftFromStartX) 
00819                + cos(rz) * (transform[4] - imuShiftFromStartY);
00820       float z1 = transform[5] * 1.05 - imuShiftFromStartZ;
00821 
00822       float x2 = x1;
00823       float y2 = cos(rx) * y1 - sin(rx) * z1;
00824       float z2 = sin(rx) * y1 + cos(rx) * z1;
00825 
00826       tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
00827       ty = transformSum[4] - y2;
00828       tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);
00829 
00830       PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart, 
00831                         imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);
00832 
00833       transformSum[0] = rx;
00834       transformSum[1] = ry;
00835       transformSum[2] = rz;
00836       transformSum[3] = tx;
00837       transformSum[4] = ty;
00838       transformSum[5] = tz;
00839 
00840       geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);
00841 
00842       laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00843       laserOdometry.pose.pose.orientation.x = -geoQuat.y;
00844       laserOdometry.pose.pose.orientation.y = -geoQuat.z;
00845       laserOdometry.pose.pose.orientation.z = geoQuat.x;
00846       laserOdometry.pose.pose.orientation.w = geoQuat.w;
00847       laserOdometry.pose.pose.position.x = tx;
00848       laserOdometry.pose.pose.position.y = ty;
00849       laserOdometry.pose.pose.position.z = tz;
00850       pubLaserOdometry.publish(laserOdometry);
00851 
00852       laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
00853       laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
00854       laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
00855       tfBroadcaster.sendTransform(laserOdometryTrans);
00856 
00857       int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
00858       for (int i = 0; i < cornerPointsLessSharpNum; i++) {
00859         TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
00860       }
00861 
00862       int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
00863       for (int i = 0; i < surfPointsLessFlatNum; i++) {
00864         TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
00865       }
00866 
00867       frameCount++;
00868       if (frameCount >= skipFrameNum + 1) {
00869         int laserCloudFullResNum = laserCloudFullRes->points.size();
00870         for (int i = 0; i < laserCloudFullResNum; i++) {
00871           TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
00872         }
00873       }
00874 
00875       pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
00876       cornerPointsLessSharp = laserCloudCornerLast;
00877       laserCloudCornerLast = laserCloudTemp;
00878 
00879       laserCloudTemp = surfPointsLessFlat;
00880       surfPointsLessFlat = laserCloudSurfLast;
00881       laserCloudSurfLast = laserCloudTemp;
00882 
00883       laserCloudCornerLastNum = laserCloudCornerLast->points.size();
00884       laserCloudSurfLastNum = laserCloudSurfLast->points.size();
00885       if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
00886         kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
00887         kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
00888       }
00889 
00890       if (frameCount >= skipFrameNum + 1) {
00891         frameCount = 0;
00892 
00893         sensor_msgs::PointCloud2 laserCloudCornerLast2;
00894         pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
00895         laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00896         laserCloudCornerLast2.header.frame_id = "/camera";
00897         pubLaserCloudCornerLast.publish(laserCloudCornerLast2);
00898 
00899         sensor_msgs::PointCloud2 laserCloudSurfLast2;
00900         pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
00901         laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00902         laserCloudSurfLast2.header.frame_id = "/camera";
00903         pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
00904 
00905         sensor_msgs::PointCloud2 laserCloudFullRes3;
00906         pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
00907         laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00908         laserCloudFullRes3.header.frame_id = "/camera";
00909         pubLaserCloudFullRes.publish(laserCloudFullRes3);
00910       }
00911 
00912       /*sensor_msgs::PointCloud2 pc12;
00913       pcl::toROSMsg(*pointSearchCornerLast, pc12);
00914       pc12.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00915       pc12.header.frame_id = "/camera";
00916       pub1.publish(pc12);
00917 
00918       sensor_msgs::PointCloud2 pc22;
00919       pcl::toROSMsg(*pointSearchSurfLast, pc22);
00920       pc22.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00921       pc22.header.frame_id = "/camera";
00922       pub2.publish(pc22);
00923 
00924       sensor_msgs::PointCloud2 pc32;
00925       pcl::toROSMsg(*pointProjCornerLast, pc32);
00926       pc32.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00927       pc32.header.frame_id = "/camera";
00928       pub3.publish(pc32);
00929 
00930       sensor_msgs::PointCloud2 pc42;
00931       pcl::toROSMsg(*pointProjSurfLast, pc42);
00932       pc42.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
00933       pc42.header.frame_id = "/camera";
00934       pub4.publish(pc42);*/
00935     }
00936 
00937     status = ros::ok();
00938     rate.sleep();
00939   }
00940 
00941   return 0;
00942 }
