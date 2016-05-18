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
00027 const int systemDelay = 20;
00028 int systemInitCount = 0;
00029 bool systemInited = false;
00030 
00031 pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
00032 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
00033 pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
00034 pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());
00035 pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
00036 pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());
00037 pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>());
00038 pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZI>());
00039 pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>(4, 1));
00040 pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudScans[16];
00041 
00042 float cloudCurvature[40000];
00043 int cloudSortInd[40000];
00044 int cloudNeighborPicked[40000];
00045 int cloudLabel[40000];
00046 
00047 int scanStartInd[16];
00048 int scanEndInd[16];
00049 
00050 int imuPointerFront = 0;
00051 int imuPointerLast = -1;
00052 const int imuQueLength = 200;
00053 
00054 float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
00055 float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
00056 
00057 float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
00058 float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;
00059 
00060 float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
00061 float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;
00062 
00063 float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
00064 float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;
00065 
00066 double imuTime[imuQueLength] = {0};
00067 float imuRoll[imuQueLength] = {0};
00068 float imuPitch[imuQueLength] = {0};
00069 float imuYaw[imuQueLength] = {0};
00070 
00071 float imuAccX[imuQueLength] = {0};
00072 float imuAccY[imuQueLength] = {0};
00073 float imuAccZ[imuQueLength] = {0};
00074 
00075 float imuVeloX[imuQueLength] = {0};
00076 float imuVeloY[imuQueLength] = {0};
00077 float imuVeloZ[imuQueLength] = {0};
00078 
00079 float imuShiftX[imuQueLength] = {0};
00080 float imuShiftY[imuQueLength] = {0};
00081 float imuShiftZ[imuQueLength] = {0};
00082 
00083 ros::Publisher* pubLaserCloudPointer;
00084 ros::Publisher* pubCornerPointsSharpPointer;
00085 ros::Publisher* pubCornerPointsLessSharpPointer;
00086 ros::Publisher* pubSurfPointsFlatPointer;
00087 ros::Publisher* pubSurfPointsLessFlatPointer;
00088 ros::Publisher* pubImuTransPointer;
00089 
00090 void ShiftToStartIMU(float pointTime)
00091 {
00092   imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
00093   imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
00094   imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;
00095 
00096   float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
00097   float y1 = imuShiftFromStartYCur;
00098   float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;
00099 
00100   float x2 = x1;
00101   float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
00102   float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;
00103 
00104   imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
00105   imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
00106   imuShiftFromStartZCur = z2;
00107 }
00108 
00109 void VeloToStartIMU()
00110 {
00111   imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
00112   imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
00113   imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;
00114 
00115   float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
00116   float y1 = imuVeloFromStartYCur;
00117   float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;
00118 
00119   float x2 = x1;
00120   float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
00121   float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;
00122 
00123   imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
00124   imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
00125   imuVeloFromStartZCur = z2;
00126 }
00127 
00128 void TransformToStartIMU(pcl::PointXYZI *p)
00129 {
00130   float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
00131   float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
00132   float z1 = p->z;
00133 
00134   float x2 = x1;
00135   float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
00136   float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;
00137 
00138   float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
00139   float y3 = y2;
00140   float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;
00141 
00142   float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
00143   float y4 = y3;
00144   float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;
00145 
00146   float x5 = x4;
00147   float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
00148   float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;
00149 
00150   p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
00151   p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
00152   p->z = z5 + imuShiftFromStartZCur;
00153 }
00154 
00155 void AccumulateIMUShift()
00156 {
00157   float roll = imuRoll[imuPointerLast];
00158   float pitch = imuPitch[imuPointerLast];
00159   float yaw = imuYaw[imuPointerLast];
00160   float accX = imuAccX[imuPointerLast];
00161   float accY = imuAccY[imuPointerLast];
00162   float accZ = imuAccZ[imuPointerLast];
00163 
00164   float x1 = cos(roll) * accX - sin(roll) * accY;
00165   float y1 = sin(roll) * accX + cos(roll) * accY;
00166   float z1 = accZ;
00167 
00168   float x2 = x1;
00169   float y2 = cos(pitch) * y1 - sin(pitch) * z1;
00170   float z2 = sin(pitch) * y1 + cos(pitch) * z1;
00171 
00172   accX = cos(yaw) * x2 + sin(yaw) * z2;
00173   accY = y2;
00174   accZ = -sin(yaw) * x2 + cos(yaw) * z2;
00175 
00176   int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
00177   double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
00178   if (timeDiff < 0.1) {
00179 
00180     imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
00181                               + accX * timeDiff * timeDiff / 2;
00182     imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
00183                               + accY * timeDiff * timeDiff / 2;
00184     imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
00185                               + accZ * timeDiff * timeDiff / 2;
00186 
00187     imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
00188     imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
00189     imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
00190   }
00191 }
00192 
00193 void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
00194 {
00195   if (!systemInited) {
00196     systemInitCount++;
00197     if (systemInitCount >= systemDelay) {
00198       systemInited = true;
00199     }
00200     return;
00201   }
00202   
00203   double timeScanCur = laserCloudIn2->header.stamp.toSec();
00204 
00205   pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
00206   int cloudSize = laserCloudIn->points.size();
00207 
00208   float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
00209   float endOri = -atan2(laserCloudIn->points[cloudSize - 1].y, 
00210                         laserCloudIn->points[cloudSize - 1].x) + 2 * PI;
00211 
00212   if (endOri - startOri > 3 * PI) {
00213     endOri -= 2 * PI;
00214   } else if (endOri - startOri < PI) {
00215     endOri += 2 * PI;
00216   }
00217 
00218   bool halfPassed = false;
00219   pcl::PointXYZI point;
00220   for (int i = 0; i < cloudSize; i++) {
00221     point.x = laserCloudIn->points[i].y;
00222     point.y = laserCloudIn->points[i].z;
00223     point.z = laserCloudIn->points[i].x;
00224 
00225     float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / PI;
00226     int scanID = int(0.75 * angle + 0.5) + 7;
00227     if (angle < 0) {
00228       scanID--;
00229     }
00230 
00231     float ori = -atan2(point.x, point.z);
00232     if (!halfPassed) {
00233       if (ori < startOri - PI / 2) {
00234         ori += 2 * PI;
00235       } else if (ori > startOri + PI * 3 / 2) {
00236         ori -= 2 * PI;
00237       }
00238 
00239       if (ori - startOri > PI) {
00240         halfPassed = true;
00241       }
00242     } else {
00243       ori += 2 * PI;
00244 
00245       if (ori < endOri - PI * 3 / 2) {
00246         ori += 2 * PI;
00247       } else if (ori > endOri + PI / 2) {
00248         ori -= 2 * PI;
00249       } 
00250     }
00251 
00252     float relTime = (ori - startOri) / (endOri - startOri);
00253     point.intensity = scanID + 0.1 * relTime;
00254 
00255     if (imuPointerLast >= 0) {
00256       float pointTime = relTime * scanPeriod;
00257       while (imuPointerFront != imuPointerLast) {
00258         if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
00259           break;
00260         }
00261         imuPointerFront = (imuPointerFront + 1) % imuQueLength;
00262       }
00263 
00264       if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
00265         imuRollCur = imuRoll[imuPointerFront];
00266         imuPitchCur = imuPitch[imuPointerFront];
00267         imuYawCur = imuYaw[imuPointerFront];
00268 
00269         imuVeloXCur = imuVeloX[imuPointerFront];
00270         imuVeloYCur = imuVeloY[imuPointerFront];
00271         imuVeloZCur = imuVeloZ[imuPointerFront];
00272 
00273         imuShiftXCur = imuShiftX[imuPointerFront];
00274         imuShiftYCur = imuShiftY[imuPointerFront];
00275         imuShiftZCur = imuShiftZ[imuPointerFront];
00276       } else {
00277         int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
00278         float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
00279                          / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
00280         float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
00281                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
00282 
00283         imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
00284         imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
00285         if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
00286           imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
00287         } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
00288           imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
00289         } else {
00290           imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
00291         }
00292 
00293         imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
00294         imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
00295         imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;
00296 
00297         imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
00298         imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
00299         imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
00300       }
00301 
00302       if (i == 0) {
00303         imuRollStart = imuRollCur;
00304         imuPitchStart = imuPitchCur;
00305         imuYawStart = imuYawCur;
00306 
00307         imuVeloXStart = imuVeloXCur;
00308         imuVeloYStart = imuVeloYCur;
00309         imuVeloZStart = imuVeloZCur;
00310 
00311         imuShiftXStart = imuShiftXCur;
00312         imuShiftYStart = imuShiftYCur;
00313         imuShiftZStart = imuShiftZCur;
00314       } else {
00315         ShiftToStartIMU(pointTime);
00316         VeloToStartIMU();
00317         TransformToStartIMU(&point);
00318       }
00319     }
00320 
00321     laserCloudScans[scanID]->push_back(point);
00322   }
00323 
00324   for (int i = 0; i < 16; i++) {
00325     *laserCloud += *laserCloudScans[i];
00326   }
00327 
00328   int scanCount = -1;
00329   for (int i = 5; i < cloudSize - 5; i++) {
00330     float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
00331                 + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
00332                 + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
00333                 + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
00334                 + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
00335                 + laserCloud->points[i + 5].x;
00336     float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
00337                 + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
00338                 + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
00339                 + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
00340                 + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
00341                 + laserCloud->points[i + 5].y;
00342     float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
00343                 + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
00344                 + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
00345                 + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
00346                 + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
00347                 + laserCloud->points[i + 5].z;
00348     
00349     cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
00350     cloudSortInd[i] = i;
00351     cloudNeighborPicked[i] = 0;
00352     cloudLabel[i] = 0;
00353 
00354     if (int(laserCloud->points[i].intensity) != scanCount) {
00355       scanCount = int(laserCloud->points[i].intensity);
00356 
00357       if (scanCount > 0) {
00358         scanStartInd[scanCount] = i + 5;
00359         scanEndInd[scanCount - 1] = i - 5;
00360       }
00361     }
00362   }
00363   scanStartInd[0] = 5;
00364   scanEndInd[15] = cloudSize - 5;
00365 
00366   for (int i = 5; i < cloudSize - 6; i++) {
00367     float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
00368     float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
00369     float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
00370     float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;
00371 
00372     if (diff > 0.1) {
00373 
00374       float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
00375                      laserCloud->points[i].y * laserCloud->points[i].y +
00376                      laserCloud->points[i].z * laserCloud->points[i].z);
00377 
00378       float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
00379                      laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
00380                      laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
00381 
00382       if (depth1 > depth2) {
00383         diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
00384         diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
00385         diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;
00386 
00387         if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
00388           cloudNeighborPicked[i - 5] = 1;
00389           cloudNeighborPicked[i - 4] = 1;
00390           cloudNeighborPicked[i - 3] = 1;
00391           cloudNeighborPicked[i - 2] = 1;
00392           cloudNeighborPicked[i - 1] = 1;
00393           cloudNeighborPicked[i] = 1;
00394         }
00395       } else {
00396         diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
00397         diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
00398         diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;
00399 
00400         if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
00401           cloudNeighborPicked[i + 1] = 1;
00402           cloudNeighborPicked[i + 2] = 1;
00403           cloudNeighborPicked[i + 3] = 1;
00404           cloudNeighborPicked[i + 4] = 1;
00405           cloudNeighborPicked[i + 5] = 1;
00406           cloudNeighborPicked[i + 6] = 1;
00407         }
00408       }
00409     }
00410 
00411     float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
00412     float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
00413     float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
00414     float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
00415 
00416     float dis = laserCloud->points[i].x * laserCloud->points[i].x
00417               + laserCloud->points[i].y * laserCloud->points[i].y
00418               + laserCloud->points[i].z * laserCloud->points[i].z;
00419 
00420     if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
00421       cloudNeighborPicked[i] = 1;
00422     }
00423   }
00424 
00425   for (int i = 0; i < 16; i++) {
00426     surfPointsLessFlatScan->clear();
00427     for (int j = 0; j < 6; j++) {
00428       int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
00429       int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;
00430 
00431       for (int k = sp + 1; k <= ep; k++) {
00432         for (int l = k; l >= sp + 1; l--) {
00433           if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
00434             int temp = cloudSortInd[l - 1];
00435             cloudSortInd[l - 1] = cloudSortInd[l];
00436             cloudSortInd[l] = temp;
00437           }
00438         }
00439       }
00440 
00441       int largestPickedNum = 0;
00442       for (int k = ep; k >= sp; k--) {
00443         int ind = cloudSortInd[k];
00444         if (cloudNeighborPicked[ind] == 0 &&
00445             cloudCurvature[ind] > 0.1) {
00446         
00447           largestPickedNum++;
00448           if (largestPickedNum <= 2) {
00449             cloudLabel[ind] = 2;
00450             cornerPointsSharp->push_back(laserCloud->points[ind]);
00451             cornerPointsLessSharp->push_back(laserCloud->points[ind]);
00452           } else if (largestPickedNum <= 20) {
00453             cloudLabel[ind] = 1;
00454             cornerPointsLessSharp->push_back(laserCloud->points[ind]);
00455           } else {
00456             break;
00457           }
00458 
00459           cloudNeighborPicked[ind] = 1;
00460           for (int l = 1; l <= 5; l++) {
00461             float diffX = laserCloud->points[ind + l].x 
00462                         - laserCloud->points[ind + l - 1].x;
00463             float diffY = laserCloud->points[ind + l].y 
00464                         - laserCloud->points[ind + l - 1].y;
00465             float diffZ = laserCloud->points[ind + l].z 
00466                         - laserCloud->points[ind + l - 1].z;
00467             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
00468               break;
00469             }
00470 
00471             cloudNeighborPicked[ind + l] = 1;
00472           }
00473           for (int l = -1; l >= -5; l--) {
00474             float diffX = laserCloud->points[ind + l].x 
00475                         - laserCloud->points[ind + l + 1].x;
00476             float diffY = laserCloud->points[ind + l].y 
00477                         - laserCloud->points[ind + l + 1].y;
00478             float diffZ = laserCloud->points[ind + l].z 
00479                         - laserCloud->points[ind + l + 1].z;
00480             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
00481               break;
00482             }
00483 
00484             cloudNeighborPicked[ind + l] = 1;
00485           }
00486         }
00487       }
00488 
00489       int smallestPickedNum = 0;
00490       for (int k = sp; k <= ep; k++) {
00491         int ind = cloudSortInd[k];
00492         if (cloudNeighborPicked[ind] == 0 &&
00493             cloudCurvature[ind] < 0.1) {
00494 
00495           cloudLabel[ind] = -1;
00496           surfPointsFlat->push_back(laserCloud->points[ind]);
00497 
00498           smallestPickedNum++;
00499           if (smallestPickedNum >= 4) {
00500             break;
00501           }
00502 
00503           cloudNeighborPicked[ind] = 1;
00504           for (int l = 1; l <= 5; l++) {
00505             float diffX = laserCloud->points[ind + l].x 
00506                         - laserCloud->points[ind + l - 1].x;
00507             float diffY = laserCloud->points[ind + l].y 
00508                         - laserCloud->points[ind + l - 1].y;
00509             float diffZ = laserCloud->points[ind + l].z 
00510                         - laserCloud->points[ind + l - 1].z;
00511             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
00512               break;
00513             }
00514 
00515             cloudNeighborPicked[ind + l] = 1;
00516           }
00517           for (int l = -1; l >= -5; l--) {
00518             float diffX = laserCloud->points[ind + l].x 
00519                         - laserCloud->points[ind + l + 1].x;
00520             float diffY = laserCloud->points[ind + l].y 
00521                         - laserCloud->points[ind + l + 1].y;
00522             float diffZ = laserCloud->points[ind + l].z 
00523                         - laserCloud->points[ind + l + 1].z;
00524             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
00525               break;
00526             }
00527 
00528             cloudNeighborPicked[ind + l] = 1;
00529           }
00530         }
00531       }
00532 
00533       for (int k = sp; k <= ep; k++) {
00534         if (cloudLabel[k] <= 0) {
00535           surfPointsLessFlatScan->push_back(laserCloud->points[k]);
00536         }
00537       }
00538     }
00539 
00540     surfPointsLessFlatScanDS->clear();
00541     pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
00542     downSizeFilter.setInputCloud(surfPointsLessFlatScan);
00543     downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
00544     downSizeFilter.filter(*surfPointsLessFlatScanDS);
00545 
00546     *surfPointsLessFlat += *surfPointsLessFlatScanDS;
00547   }
00548 
00549   sensor_msgs::PointCloud2 laserCloud2;
00550   pcl::toROSMsg(*laserCloud, laserCloud2);
00551   laserCloud2.header.stamp = laserCloudIn2->header.stamp;
00552   laserCloud2.header.frame_id = "/camera";
00553   pubLaserCloudPointer->publish(laserCloud2);
00554 
00555   sensor_msgs::PointCloud2 cornerPointsSharp2;
00556   pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharp2);
00557   cornerPointsSharp2.header.stamp = laserCloudIn2->header.stamp;
00558   cornerPointsSharp2.header.frame_id = "/camera";
00559   pubCornerPointsSharpPointer->publish(cornerPointsSharp2);
00560 
00561   sensor_msgs::PointCloud2 cornerPointsLessSharp2;
00562   pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharp2);
00563   cornerPointsLessSharp2.header.stamp = laserCloudIn2->header.stamp;
00564   cornerPointsLessSharp2.header.frame_id = "/camera";
00565   pubCornerPointsLessSharpPointer->publish(cornerPointsLessSharp2);
00566 
00567   sensor_msgs::PointCloud2 surfPointsFlat2;
00568   pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
00569   surfPointsFlat2.header.stamp = laserCloudIn2->header.stamp;
00570   surfPointsFlat2.header.frame_id = "/camera";
00571   pubSurfPointsFlatPointer->publish(surfPointsFlat2);
00572 
00573   sensor_msgs::PointCloud2 surfPointsLessFlat2;
00574   pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
00575   surfPointsLessFlat2.header.stamp = laserCloudIn2->header.stamp;
00576   surfPointsLessFlat2.header.frame_id = "/camera";
00577   pubSurfPointsLessFlatPointer->publish(surfPointsLessFlat2);
00578 
00579   imuTrans->points[0].x = imuPitchStart;
00580   imuTrans->points[0].y = imuYawStart;
00581   imuTrans->points[0].z = imuRollStart;
00582 
00583   imuTrans->points[1].x = imuPitchCur;
00584   imuTrans->points[1].y = imuYawCur;
00585   imuTrans->points[1].z = imuRollCur;
00586 
00587   imuTrans->points[2].x = imuShiftFromStartXCur;
00588   imuTrans->points[2].y = imuShiftFromStartYCur;
00589   imuTrans->points[2].z = imuShiftFromStartZCur;
00590 
00591   imuTrans->points[3].x = imuVeloFromStartXCur;
00592   imuTrans->points[3].y = imuVeloFromStartYCur;
00593   imuTrans->points[3].z = imuVeloFromStartZCur;
00594 
00595   sensor_msgs::PointCloud2 imuTrans2;
00596   pcl::toROSMsg(*imuTrans, imuTrans2);
00597   imuTrans2.header.stamp = laserCloudIn2->header.stamp;
00598   imuTrans2.header.frame_id = "/camera";
00599   pubImuTransPointer->publish(imuTrans2);
00600 
00601   laserCloudIn->clear();
00602   laserCloud->clear();
00603   cornerPointsSharp->clear();
00604   cornerPointsLessSharp->clear();
00605   surfPointsFlat->clear();
00606   surfPointsLessFlat->clear();
00607 
00608   for (int i = 0; i < 16; i++) {
00609     laserCloudScans[i]->points.clear();
00610   }
00611 }
00612 
00613 void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
00614 {
00615   double roll, pitch, yaw;
00616   tf::Quaternion orientation;
00617   tf::quaternionMsgToTF(imuIn->orientation, orientation);
00618   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
00619 
00620   float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
00621   float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
00622   float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;
00623 
00624   imuPointerLast = (imuPointerLast + 1) % imuQueLength;
00625 
00626   imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
00627   imuRoll[imuPointerLast] = roll;
00628   imuPitch[imuPointerLast] = pitch;
00629   imuYaw[imuPointerLast] = yaw;
00630   imuAccX[imuPointerLast] = accX;
00631   imuAccY[imuPointerLast] = accY;
00632   imuAccZ[imuPointerLast] = accZ;
00633 
00634   AccumulateIMUShift();
00635 }
00636 
00637 int main(int argc, char** argv)
00638 {
00639   ros::init(argc, argv, "scanRegistration");
00640   ros::NodeHandle nh;
00641 
00642   for (int i = 0; i < 16; i++) {
00643     laserCloudScans[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
00644   }
00645 
00646   ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
00647                                   ("/velodyne_cloud", 2, laserCloudHandler);
00648 
00649   ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);
00650 
00651   ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> 
00652                                  ("/velodyne_cloud_2", 2);
00653 
00654   ros::Publisher pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2> 
00655                                         ("/laser_cloud_sharp", 2);
00656 
00657   ros::Publisher pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2> 
00658                                             ("/laser_cloud_less_sharp", 2);
00659 
00660   ros::Publisher pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2> 
00661                                        ("/laser_cloud_flat", 2);
00662 
00663   ros::Publisher pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2> 
00664                                            ("/laser_cloud_less_flat", 2);
00665 
00666   ros::Publisher pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);
00667 
00668   pubLaserCloudPointer = &pubLaserCloud;
00669   pubCornerPointsSharpPointer = &pubCornerPointsSharp;
00670   pubCornerPointsLessSharpPointer = &pubCornerPointsLessSharp;
00671   pubSurfPointsFlatPointer = &pubSurfPointsFlat;
00672   pubSurfPointsLessFlatPointer = &pubSurfPointsLessFlat;
00673   pubImuTransPointer = &pubImuTrans;
00674 
00675   ros::spin();
00676 
00677   return 0;
00678 }

