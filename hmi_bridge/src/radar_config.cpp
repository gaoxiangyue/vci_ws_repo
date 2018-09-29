#include "ros/ros.h"
#include "radar_config.h"
#include "CanUtils.hpp"
#include "Common.hpp"
#include <memory>
#include <cmath>

namespace radar_config
{
using namespace bosch_mrr;
parse_frames::parse_frames(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  //,timesync_(nh,nh_private)
{
  READ_PARAM_BEGIN;
  /* node parameter*/
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, fps_radar_tx_, 50);
  READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, can_radar_rx_, "/can_rx");
  READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, can_radar_tx_, "/can_tx");
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, config_times_, 1);
  READ_PRIVATE_PARAM_WITH_DEFAULT(std::string, can_frame_id_, "radar_config");

  /* radar config param*/
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, radar_type_, 1);
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, radar_id_old_, 0);
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, radar_id_new_, 0);
  READ_PRIVATE_PARAM_WITH_DEFAULT(int, output_type_, 1);//<!--0:none 1:object 2:cluster-->
  
  /* publishers*/
  pub_can_radar_ = nh.advertise<can_msgs::Frame>(can_radar_tx_, 1);

  /* subscribers */
  sub_can_radar_ = nh.subscribe(can_radar_rx_, 100, &parse_frames::processRadarMsg,this);

  /* scheduling publishers*/
  timer_pub_can_ = nh.createWallTimer(ros::WallDuration(1./fps_radar_tx_), &parse_frames::publishCanFrame, this);

  //last_timestamp_=ros::Time::now();
}

parse_frames::~parse_frames()
{

}

void parse_frames::processRadarMsg(const can_msgs::Frame::ConstPtr &can_msg)
{
  //DEBUG_PRINT_VAR(can_msg->id);
  if(radar_type_==1)//for ars408-21
  {
      parseArsStatus(radar_id_old_,can_msg); //ars
      parseArsStatus(radar_id_new_,can_msg); //ars
  }
  else if(radar_type_==2)
  {
      parseSrrStatus(radar_id_old_,can_msg); //srr
      parseSrrStatus(radar_id_new_,can_msg); //srr

  }
  else
  ;
}

bool parse_frames::parseArsStatus(uint32_t radar_id, const can_msgs::Frame::ConstPtr& msg)
{
  //for Continental ARS 408-21
  //message ID=message ID0+sensor ID*0x10
   momenta_msgs::ArsObject object;
   if( msg->id == 0x201+radar_id*0x10){
        // RadarStatus
        int RadarState_NVMReadStatus=bosch_mrr::extract_bits<int>(*msg, 6, 1, 1, 0, true, false);
        int RadarState_NVMwriteStatus=bosch_mrr::extract_bits<int>(*msg, 7, 1, 1, 0, true, false);
        int RadarState_MaxDistanceCfg=bosch_mrr::extract_bits<int>(*msg, 15, 10, 1, 0, true, false);
        int RadarState_Persistent_Error=bosch_mrr::extract_bits<int>(*msg, 21, 1, 1, 0, true, false);
        int RadarState_Interference=bosch_mrr::extract_bits<int>(*msg, 20, 1, 1, 0, true, false);
        int RadarState_Temperature_Error=bosch_mrr::extract_bits<int>(*msg, 19, 1, 1, 0, true, false);
        int RadarState_Voltage_Error=bosch_mrr::extract_bits<int>(*msg, 18, 1, 1, 0, true, false);
        int RadarState_SensorID=bosch_mrr::extract_bits<int>(*msg, 34, 3, 1, 0, true, false);
        int RadarState_SortIndex=bosch_mrr::extract_bits<int>(*msg, 38, 3, 1, 0, true, false);
        int RadarState_RadarPowerCfg=bosch_mrr::extract_bits<int>(*msg, 25, 3, 1, 0, true, false);
        int RadarState_CtrlRelayCfg=bosch_mrr::extract_bits<int>(*msg, 41, 1, 1, 0, true, false);
        int RadarState_OutputTypeCfg=bosch_mrr::extract_bits<int>(*msg, 43, 2, 1, 0, true, false);
        int RadarState_SendQualityCfg=bosch_mrr::extract_bits<int>(*msg, 44, 1, 1, 0, true, false);
        int RadarState_SendExtInfoCfg=bosch_mrr::extract_bits<int>(*msg, 45, 1, 1, 0, true, false);
        int RadarState_MotionRxState=bosch_mrr::extract_bits<int>(*msg, 47, 2, 1, 0, true, false);
        int RadarState_RCS_Threshold=bosch_mrr::extract_bits<int>(*msg, 60, 3, 1, 0, true, false);
        ROS_INFO_VAR(radar_id);
        ROS_INFO_VAR(RadarState_MotionRxState);
        ROS_INFO_VAR(RadarState_NVMwriteStatus);
        ROS_INFO_VAR(RadarState_MaxDistanceCfg);
        ROS_INFO_VAR(RadarState_Persistent_Error);
        ROS_INFO_VAR(RadarState_Interference);
        ROS_INFO_VAR(RadarState_Temperature_Error);
        ROS_INFO_VAR(RadarState_Voltage_Error);
        ROS_INFO_VAR(RadarState_SensorID);
        ROS_INFO_VAR(RadarState_SortIndex);
        ROS_INFO_VAR(RadarState_RadarPowerCfg);
        ROS_INFO_VAR(RadarState_OutputTypeCfg);
        ROS_INFO_VAR(RadarState_SendQualityCfg);
        ROS_INFO_VAR(RadarState_SendExtInfoCfg);
        ROS_INFO_VAR(RadarState_MotionRxState);
        ROS_INFO_VAR(RadarState_RCS_Threshold);
        if(RadarState_NVMwriteStatus)
        {
            config_times_=0;
            ROS_INFO("Ars408-21 RadarState_NVMwriteStatus Config successfull......");
        }
        
   }
   else
     return false;
}

bool parse_frames::parseSrrStatus(uint32_t radar_id, const can_msgs::Frame::ConstPtr& msg)
{
    //for Continental SRR 208-21
    //message ID=message ID0+sensor ID*0x10
    momenta_msgs::ArsObject object;
    if( msg->id == 0x60A+radar_id*0x10){
        // RadarStatus
        int ACTL_Mode=bosch_mrr::extract_bits<int>(*msg, 5, 6, 1, 0, true, false);
        int Radar_Cfg_Status=bosch_mrr::extract_bits<int>(*msg, 15, 4, 1, 0, true, false);
        int RadarSt_RollCount=bosch_mrr::extract_bits<int>(*msg, 9, 2, 1, 0, true, false);
        ROS_INFO_VAR(radar_id);
        ROS_INFO_VAR(ACTL_Mode);
        ROS_INFO_VAR(Radar_Cfg_Status);
        ROS_INFO_VAR(RadarSt_RollCount);
        if(Radar_Cfg_Status==0)
        {
            ROS_INFO("Srr208-21 Last learned cfg used/default value after sensor startup");
        }
        else if(Radar_Cfg_Status==1)
        {
            ROS_INFO("Srr208-21 New cfg is used/Config successfull......");
        }
        else if(Radar_Cfg_Status==2)
        {
            ROS_INFO("Srr208-21 Desired ID already used");
        }
        else if(Radar_Cfg_Status==3)
        {
            ROS_INFO("Srr208-21 Default value Nvm reading failed");
        }
        else 
        {
            ROS_INFO("Srr208-21 Nvm writing failed!");
        }
    }
    else
        return false;
}

void parse_frames::publishCanFrame(const ros::WallTimerEvent& event)
{
  if(config_times_>0)
    config_times_--;
  else
    return;
  if(radar_type_==1)//for ars408-21
  {
      configArsRadar(radar_id_old_,radar_id_new_,output_type_);
  }
  else if(radar_type_==2)//for srr208-21
  {
      configSrrRadar(radar_id_old_,radar_id_new_,output_type_);
  }
  else
      ROS_INFO("not support radar_type_:%d",radar_type_);
}

void parse_frames::configSrrRadar(uint32_t old_id,uint32_t new_id, uint8_t output_type)
{
    //for srr20821
    can_msgs::Frame config_frame;
    config_frame.id = 0x200+old_id*0x10;
    config_frame.dlc=8;
    config_frame.header.frame_id=can_frame_id_;
    int Radar_Output_Type_Valid=1;        //Valid
    int Radar_Output_Type=output_type-1;  //SendTracks0 SendCluster1
    int Radar_ID_Valid=1;                 //Valid
    uint32_t Radar_ID=new_id;             //NEW_ID
    // Frame, start_bit, bit_len, factor, offset, data, big_endian, signed
    pack_bits<int>(config_frame, 56, 1, 1, 0, Radar_Output_Type_Valid, true, false); 
    pack_bits<int>(config_frame, 5, 2, 1, 0, Radar_Output_Type, true, false); 
    pack_bits<int>(config_frame, 57, 1, 1, 0, Radar_ID_Valid, true, false);
    pack_bits<uint32_t>(config_frame, 3, 4, 1, 0, Radar_ID, true, false);

    pub_can_radar_.publish(config_frame);
    ROS_INFO("configsSrrRadar publish...id:%d-->%d|type %d",old_id,new_id,output_type);
}

void parse_frames::configArsRadar(uint32_t old_id,uint32_t new_id, uint8_t output_type)
{
    //for srr20821
    can_msgs::Frame config_frame;
    config_frame.id = 0x200+old_id*0x10;
    config_frame.dlc=8;
    config_frame.header.frame_id=can_frame_id_;
    int RadarCfg_MaxDistance_valid=0;
    int RadarCfg_SensorID_valid=1;      
    int RadarCfg_RadarPower_valid=0;
    int RadarCfg_OutputType_valid=1;    
    int RadarCfg_SendQuality_valid=1;   
    int RadarCfg_SendExtInfo_valid=1;   
    int RadarCfg_SortIndex_valid=0;
    int RadarCfg_StoreInNVM_valid=1;    
    int RadarCfg_MaxDistance=196;
    int RadarCfg_SensorID=new_id;
    int RadarCfg_OutputType=output_type;//0none 1obj 2cluster
    int RadarCfg_RadarPower=0;          //Standard
    int RadarCfg_CtrlRelay_valid=0;
    int RadarCfg_CtrlRelay=0;
    int RadarCfg_SendQuality=1;
    int RadarCfg_SendExtInfo=1;
    int RadarCfg_SortIndex=1;
    int RadarCfg_StoreInNVM=1;          //Store
    int RadarCfg_RCS_Threshold_valid=0; 
    int RadarCfg_RCS_Threshold=0;
    // Frame, start_bit, bit_len, factor, offset, data, big_endian, signed
    pack_bits<int>(config_frame, 0, 1, 1, 0, RadarCfg_MaxDistance_valid, true, false); 
    pack_bits<int>(config_frame, 1, 1, 1, 0, RadarCfg_SensorID_valid, true, false);
    pack_bits<int>(config_frame, 2, 1, 1, 0, RadarCfg_RadarPower_valid, true, false);
    pack_bits<int>(config_frame, 3, 1, 1, 0, RadarCfg_OutputType_valid, true, false);
    pack_bits<int>(config_frame, 4, 1, 1, 0, RadarCfg_SendQuality_valid, true, false);
    pack_bits<int>(config_frame, 5, 1, 1, 0, RadarCfg_SendExtInfo_valid, true, false);
    pack_bits<int>(config_frame, 6, 1, 1, 0, RadarCfg_SortIndex_valid, true, false);
    pack_bits<int>(config_frame, 7, 1, 1, 0, RadarCfg_StoreInNVM_valid, true, false);
    pack_bits<int>(config_frame, 15, 10, 2, 0, RadarCfg_MaxDistance, true, false); 
    pack_bits<int>(config_frame, 34, 3, 1, 0, RadarCfg_SensorID, true, false);
    pack_bits<int>(config_frame, 36, 2, 1, 0, RadarCfg_OutputType, true, false);
    pack_bits<int>(config_frame, 39, 3, 1, 0, RadarCfg_RadarPower, true, false);
    pack_bits<int>(config_frame, 40, 1, 1, 0, RadarCfg_CtrlRelay_valid, true, false);
    pack_bits<int>(config_frame, 41, 1, 1, 0, RadarCfg_CtrlRelay, true, false);
    pack_bits<int>(config_frame, 42, 1, 1, 0, RadarCfg_SendQuality, true, false);
    pack_bits<int>(config_frame, 43, 1, 1, 0, RadarCfg_SendExtInfo, true, false);
    pack_bits<int>(config_frame, 46, 3, 1, 0, RadarCfg_SortIndex, true, false);
    pack_bits<int>(config_frame, 47, 1, 1, 0, RadarCfg_StoreInNVM, true, false);
    pack_bits<int>(config_frame, 48, 1, 1, 0, RadarCfg_RCS_Threshold_valid, true, false);
    pack_bits<int>(config_frame, 51, 3, 1, 0, RadarCfg_RCS_Threshold, true, false);

    pub_can_radar_.publish(config_frame);
    ROS_INFO("configsArsRadar publish...%d-->id:%d|type %d",old_id,new_id,output_type);
}

}//namespace radar_config
