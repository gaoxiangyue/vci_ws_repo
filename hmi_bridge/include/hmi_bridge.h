#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H
#include "CanUtils.hpp"
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/timer.h>
#include <map>
#include <can_msgs/Frame.h>
#include <momenta_msgs/Object.h>
#include <momenta_msgs/ObjectArray.h>
#include <momenta_msgs/ArsObject.h>
#include <momenta_msgs/ArsObjectArray.h>
#include <vci_msgs/RadarObject.h>
#include <vci_msgs/RadarObjectArray.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/WheelSpeedReport.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/BrakeInfoReport.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/ThrottleInfoReport.h>
#include <dynamic_reconfigure/server.h>

namespace hmi_bridge {
//using namespace timesync;
using namespace std;

class parse_frames
{
public:
  parse_frames(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~parse_frames();
  void publishCanFrame(const ros::WallTimerEvent& event);

private:
  void processRadarMsg(const can_msgs::Frame::ConstPtr& can_msg);
  bool parseSrrStatus(uint32_t radar_id, const can_msgs::Frame::ConstPtr& msg);//for srr
  bool parseArsStatus(uint32_t radar_id, const can_msgs::Frame::ConstPtr& msg);//for ars
  void configSrrRadar(uint32_t old_id,uint32_t new_id, uint8_t output_type);
  void configArsRadar(uint32_t old_id,uint32_t new_id, uint8_t output_type);

  /* Subscribers & publishers */
     ros::Subscriber sub_can_radar_;
     ros::Publisher pub_can_radar_;

     /* ROS parameters */
         int fps_radar_tx_;         // fps publishing car info to radar
         int config_times_;
         std::string can_frame_id_;
         std::string can_radar_rx_; // radar -> driver ("can_rx")
         std::string can_radar_tx_; // driver -> radar
         int radar_type_;
         int radar_id_old_;
         int radar_id_new_;
         int output_type_;

         /* timers */
         ros::WallTimer timer_pub_can_;

         /* test */
         vci_msgs::RadarObjectArray radar_objects;

};

}//namespace hmi_bridge

#endif // hmi_bridge
