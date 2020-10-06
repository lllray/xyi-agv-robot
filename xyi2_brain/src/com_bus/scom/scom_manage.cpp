#include "scom_manage.h"

#include "scom_bsp.h"
#include "scom_function.h"

#include "../../class_tool/hfunc.h"

#define SCOM_DOUBLE_PI  6.2831853148f
#define SCOM_PI         3.1415926574f

void scom_angle_protect(float &angle_in)
{
    if(angle_in > SCOM_PI)  angle_in -= SCOM_DOUBLE_PI;
    else if(angle_in < -SCOM_PI)  angle_in += SCOM_DOUBLE_PI;
}

/// >------------------------------------------------------------------------------------
/// Scom Port

void ScomThread_Manage::SCOMInit(void)
{
    if(Scom_Init()){
      scom_enable = true;
      ROS_INFO("[ttyusb_com]: Scom Start Success");
    }else{
      scom_enable = false;
      ROS_INFO("[ttyusb_com]: Scom Start Fail");
    }
}


void ScomThread_Manage::SCOMClose(void)
{
    scom_enable = false;

    Scom_Close();
}

/// >-----------------------------------------------------------------------------------
/// Sub_topics' functions

void ScomThread_Manage::SCOMPub_Cmdvel(const geometry_msgs::Twist& cmd_vel)
{
    if(scom_enable)
    {
        std::memcpy(ScomSdta, NormalSDta, CommandLenth);

        PackCmd_VelocityCmand(cmd_vel, ScomSdta);

        SCom_WriteChar(ScomSdta);
    }
}

void ScomThread_Manage::SCOMPub_BeepAck(void)
{
    if(scom_enable)
    {
        std::memcpy(ScomSdta, NormalSDta, CommandLenth);

        PackCmd_BeepCmand(ScomSdta);

        SCom_WriteChar(ScomSdta);
    }
}

void func_euler2quaternion(Eigen::Vector3f &in_euler, Eigen::Quaternionf &out_q);

bool ScomThread_Manage::SCOMSub(geometry_msgs::Pose &RTOSOdom,
                                geometry_msgs::Point32 &RTOSPosi,
                                geometry_msgs::Twist &RTOSTwist,
                                geometry_msgs::Pose &s_orientaton)
{
    static CMAND_Robot2TK1_ID deal_Id;
    static unsigned int deal_ContentLen;
    char deal_Content[CommandLenth];

    if(scom_enable)
    {
        SCom_ReadChar(ScomRdta);
        Scom_Flush();

        //cout << "[ INFO ] SCOM Receive Once" << endl;
        if(F_PackCommand_ExtractMes(ScomRdta, &deal_Id, deal_Content, &deal_ContentLen))
        {
            // [Id] -------------------------------------------------
            if((unsigned char)(deal_Id) == Ack_Cmand){
              ROS_INFO("[Ack]: Received");
              std_msgs::Int8 AckContent;
              AckContent.data = deal_Content[0];

              TopicsPublish_AckCmand(AckContent);
            }

            // [Id] -------------------------------------------------
            if((unsigned char)(deal_Id) == ReturnDta_Cmand_FrameDta){

              static float Frame_Dta[7];

              static bool  Fc_flag = true;
              static float Frame_DtaC[7];
              Pointer_Char2Float(deal_Content, 28, Frame_Dta);

              if(Fc_flag)
              {
                  memcpy(Frame_DtaC, Frame_Dta, sizeof(float)*7);
                  Fc_flag = false;
              }

              RTOSPosi.x = Frame_Dta[0] - Frame_DtaC[0];
              RTOSPosi.y = Frame_Dta[1] - Frame_DtaC[1];
              RTOSPosi.z = 0;

              Eigen::Vector3f posi_euler, ori_euler;
              ori_euler.data()[0] = (Frame_Dta[2] - Frame_DtaC[2]) * 0.017453f,
              ori_euler.data()[1] = (Frame_Dta[3] - Frame_DtaC[3]) * 0.017453f,
              ori_euler.data()[2] = (Frame_Dta[4] - Frame_DtaC[4]) * 0.017453f;

              scom_angle_protect(ori_euler.data()[0]),
               scom_angle_protect(ori_euler.data()[1]),
                scom_angle_protect(ori_euler.data()[2]);

              posi_euler.data()[0] = 0,
              posi_euler.data()[1] = 0,
              posi_euler.data()[2] = ori_euler.data()[2];

              RTOSTwist.linear.x = Frame_Dta[5];
              RTOSTwist.angular.z = Frame_Dta[6];

              RTOSOdom.position.x = RTOSPosi.x; RTOSOdom.position.y = RTOSPosi.y; RTOSOdom.position.z = 0;

              Eigen::Quaternionf out_q;
              func_euler2quaternion(posi_euler, out_q);
              RTOSOdom.orientation.w = out_q.w();
              RTOSOdom.orientation.x = out_q.x();
              RTOSOdom.orientation.y = out_q.y();
              RTOSOdom.orientation.z = out_q.z();

              func_euler2quaternion(ori_euler, out_q);
              s_orientaton.orientation.w = out_q.w();
              s_orientaton.orientation.x = out_q.x();
              s_orientaton.orientation.y = out_q.y();
              s_orientaton.orientation.z = out_q.z();

              nav_msgs::Odometry ROdomDta;
              ROdomDta.pose.pose = RTOSOdom;

              TopicsPublish_ReturnDtaCmandOdomDta(ROdomDta);          //Pub Odom
              //TopicsPublish_ReturnDtaCmandFrameDta(RTOSPosi);         //Pub Posi
              //TopicsPublish_ReturnDtaCmandFrameDtaTwist(RTOSTwist);	//Pub Velocity,Gyro_Rotate

              return true;
            }
        }
    }

    return false;
}

void func_euler2quaternion(Eigen::Vector3f &in_euler, Eigen::Quaternionf &out_q)
{
    float p,r,y;
    p = in_euler.data()[0] * 0.5f,
     r = in_euler.data()[1] * 0.5f,
      y = in_euler.data()[2] * 0.5f;

    out_q.coeffs()[3] = m_cos(r)*m_cos(p)*m_cos(y) + m_sin(r)*m_sin(p)*m_sin(y);	//When yaw: 	cos(yaw*0.5f)
    out_q.coeffs()[0] = m_sin(r)*m_cos(p)*m_cos(y) - m_cos(r)*m_sin(p)*m_sin(y);	//		0
    out_q.coeffs()[1] = m_cos(r)*m_sin(p)*m_cos(y) + m_sin(r)*m_cos(p)*m_sin(y);	//		0
    out_q.coeffs()[2] = m_cos(r)*m_cos(p)*m_sin(y) - m_sin(r)*m_sin(p)*m_cos(y);	//		sin(yaw*0.5f)

    out_q.normalize();
}












