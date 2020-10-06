#ifndef __scom_manage_H
#define __scom_manage_H

    #include "ros/ros.h"
    #include <sstream>
    #include "boost/thread.hpp"

    #include <boost/asio.hpp>
    #include <boost/bind.hpp>
    #include <iostream>

    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "geometry_msgs/Pose.h"
    #include "geometry_msgs/Twist.h"
    #include "geometry_msgs/Point32.h"

    #include "scom_config.h"

    typedef class ScomThread_Manage
    {
    public:
    ScomThread_Manage()
        {
            scom_enable = false;

            for(int i=0; i<CommandLenth; i++)
                NormalSDta[i] = 0X00;
        }

        ~ScomThread_Manage()
        {
            SCOMClose();
        }

        void SCOMInit(void);
        void SCOMClose(void);

        void SCOMPub_Cmdvel(const geometry_msgs::Twist& cmd_vel);
        void SCOMPub_BeepAck(void);

        bool SCOMSub(geometry_msgs::Pose &RTOSOdom,
                     geometry_msgs::Point32 &RTOSPosi,
                     geometry_msgs::Twist &RTOSTwist,
                     geometry_msgs::Pose &s_orientaton);

    private:
        bool scom_enable;

        char NormalSDta[CommandLenth];

        char ScomRdta[CommandLenth];
        char ScomSdta[CommandLenth];


    }ScomThread_Manage;

#endif




