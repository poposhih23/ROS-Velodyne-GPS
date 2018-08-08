// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"
#include <velodyne_pointcloud/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdlib.h>
using namespace std;

namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}






MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper,
                                             const RegistrationParams& config)
    : ScanRegistration(config),
      _systemDelay(SYSTEM_DELAY),
      _scanMapper(scanMapper)
{

};



bool MultiScanRegistration::setup(ros::NodeHandle& node,
                                  ros::NodeHandle& privateNode)
{
  if (!ScanRegistration::setup(node, privateNode)) {
    return false;
  }

  // fetch scan mapping params
  std::string lidarName;

  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
    if (!privateNode.hasParam("scanPeriod")) {
      _config.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", _config.scanPeriod);
    }
  } else {
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }


  // subscribe to input cloud topic
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}



void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (_systemDelay > 0) {
    _systemDelay--;
    return;

  }
char gpsMsg[72];

  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
//usr/include/pcl1.7/pcl conversions.h and pcl1.7/ros/conversions.h
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
//ROS_INFO("%d",laserCloudMsg->header.stamp);
memcpy(gpsMsg,&laserCloudMsg->data[1174],72);

/*for (int i =0;i<72;i++){



cerr<<laserCloudMsg->data[1207+i];//gpsMsg[i];
}cerr<<endl;*/

  process(laserCloudIn, laserCloudMsg->header.stamp,gpsMsg);


}



void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZI>& laserCloudIn,
                                    const ros::Time& scanTime,
                                    char gpsMsg[72])
{
  size_t cloudSize = laserCloudIn.size();



  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);
  //tokenize the GPS Message
              char* token;
              char* get_str[10];
              token=strtok(gpsMsg,",");
              int i=0;
              while(token != NULL){
                 get_str[i]=token ;
                //cerr<<token;
                  i++;
                token = strtok(NULL,",");
              }
//cerr << "1: " << get_str[0] << "  2: " << get_str[1] << "  3: "<<get_str[2]<<" 4: "<<get_str[3]<< "5: " << get_str[4] << "  6: " << get_str[5] << "  7: "<<get_str[6]<<endl;
//convert char into float
float gps_n=0,gps_w=0;
int time_stamp=0;
time_stamp=atoi(get_str[1]);
gps_n=atof( get_str[3]);
gps_w=atof( get_str[5]);



  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZI point;

  std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(_scanMapper.getNumberOfScanRings());

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;
    point.intensity = laserCloudIn[i].intensity;
    point.t=time_stamp;
    point.n=gps_n;
    point.w=gps_w;
//ROS_INFO("Time Stamp:%d N:%f W:%f",point.t,point.n,point.w);

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = _config.scanPeriod * (ori - startOri) / (endOri - startOri);

    //point.intensity = scanID + relTime;
    //memcpy(&point.intensity,&gpsMsg[0],1);

    // project point to the start of the sweep using corresponding IMU data
    if (hasIMUData()) {
      setIMUTransformFor(relTime);
      transformToStartIMU(point);
    }

    laserCloudScans[scanID].push_back(point);
  }

  // construct sorted full resolution cloud
  cloudSize = 0;
  for (int i = 0; i < _scanMapper.getNumberOfScanRings(); i++) {
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  // extract features
  extractFeatures();

  // publish result
  publishResult();
}

} // end namespace loam
