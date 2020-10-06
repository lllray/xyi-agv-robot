#ifndef __scom_function_H
#define __scom_function_H

  #include "ros/ros.h"
  #include "geometry_msgs/Twist.h"  
  #include "geometry_msgs/Point32.h"
  #include "std_msgs/Int8.h"
  #include "nav_msgs/Odometry.h"

  #include "scom_config.h"

  #include "scom_bsp.h"

  enum CMAND_ERRORType{InitError_Null = 0x00,
			InitError_Device = 0x01};

  enum CMAND_TK12Robot_ID{Velocity_Cmand = 0x01,
			Read_Cmand = 0x02,
            Posi_Cmand = 0x03,
            Beep_Cmand = 0x04};

  enum CMAND_Robot2TK1_ID{Ack_Cmand = 0x81, 
			ReturnDta_Cmand_FrameDta = 0x82};


  void F_Memcpy(char *CharDes, unsigned int DesBegin, char *CharOrin, unsigned int Len);
  void F_FillCommand(char *CharDes, unsigned int Len);
  void Pointer_Float2Char(float *FloatDta_in, unsigned int Len, char *CharDta_out);
  void Pointer_Char2Float(char *CharDta_in, unsigned int Len, float *FloatDta_out);

  void F_PackCommand2Robot_PackMes(enum CMAND_TK12Robot_ID IdType, char *Content, unsigned int ContentLen, char *DtaPack);
  bool F_PackCommand_ExtractMes(char *DtaPack, CMAND_Robot2TK1_ID *CMandID, char *Content, unsigned int *ContentLen);

  // CmdPack
  void PackCmd_VelocityCmand(geometry_msgs::Twist cmd_vel, char *DtaBuf);
  void PackCmd_PosiCmand(geometry_msgs::Point32 posi_cmd, char *DtaBuf);
  void PackCmd_BeepCmand(char *DtaBuf);

  // CmdTopics Pub
  void TopicsPublish_AckCmand(std_msgs::Int8 Ack);
  void TopicsPublish_ReturnDtaCmandOdomDta(nav_msgs::Odometry OdomDta);
  void TopicsPublish_ReturnDtaCmandFrameDta(geometry_msgs::Point32 PointDta);
  void TopicsPublish_ReturnDtaCmandFrameDtaTwist(geometry_msgs::Twist TwistDta);

#endif




