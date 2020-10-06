/* ********************************************************************************
 * author: chenyingbing
 * time: 20170313 21:25 - 20170601   in XIAMEN University
 * illustration:
 *      welcome to the graduation design of cyb.
 *      this work has refer to the following open-source projects / papers:
 *          1. hectormapping, ros
 *              << a flexible and scalable slam system with full 3d motion estimation >>
 *
 *          2. kdtree.cpp, gmapping, ros
 *
 *          3. << Real-time correlative scan matching >>
 *
 *          4. 4-point-unit idea from << S4PCS _4-points congruent sets for robust pairwise surface registration >>
 *
 *          ??? unfinished
 *
 *
 * *******************************************************************************/

#include "main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pkg_main");
    ros::NodeHandle nstru("~");

    /// >-----------------------------------------------------------------
    /// Get param
    bool param_sta = true;

    bool enable_param[4];
    double lsr_location[3];
    double map2odom_location[2];

    std::string workroute;

    param_sta &= nstru.getParam("enable_slam",enable_param[0]);
    param_sta &= nstru.getParam("enbale_matching",enable_param[1]);
    param_sta &= nstru.getParam("enbale_renewodom",enable_param[2]);
    param_sta &= nstru.getParam("enbale_scom",enable_param[3]);

    param_sta &= nstru.getParam("lsr_x",lsr_location[0]);
    param_sta &= nstru.getParam("lsr_y",lsr_location[1]);
    param_sta &= nstru.getParam("lsr_z",lsr_location[2]);

    param_sta &= nstru.getParam("map2odom_origin_x",map2odom_location[0]);
    param_sta &= nstru.getParam("map2odom_origin_y",map2odom_location[1]);

    if(enable_param[0])
        param_sta &= nstru.getParam("workspace_route", workroute);

    /// >-----------------------------------------------------------------
    /// Default param
    float param_data[23];
    Thread_Manage *thread_manage = new Thread_Manage;

    param_data[0] = 0.1f;               // [lsr] sigama
    param_data[1] = 0.95f;              // [lsr] z_hit
    param_data[2] = 0.05f;              // [lsr] z_rand

    // SWin_lvl0: used, lvl1: unused.

    param_data[3] = 1.25f,              // SWin_XYRange[0]
    param_data[4] = 0.1f,               // SWin_XYRange[1]

    param_data[5] = 0.25f,              // SWin_XYreso[0]
    param_data[6] = 0.1f,               // SWin_XYreso[1]

    param_data[7] = 0.008726f,          // SWin_YawRange[0]    +-0.5 degree
    param_data[8] = 0.008726f,          // SWin_YawRange[1]    +-0.5 degree

    param_data[9] = 0.008726f;          // SWin_YawReso[0]     0.5 degree
    param_data[10] = 0.008726f;         // SWin_YawReso[1]     0.5 degree

    param_data[11] = 0.1f;              // Hetormaping H_sigama for Cov estimation based on _Hessian

    if(param_sta)
    {
        ROS_INFO("*****************************************************************");
        ROS_INFO("[xyi2_brain]: Params are all prepared, init the task as you wish.");
        ROS_INFO("*****************************************************************");
        param_data[12] = lsr_location[0];       // [lsr] location_x
        param_data[13] = lsr_location[1];       // [lsr] location_y
        param_data[14] = lsr_location[2];       // [lsr] location_yaw

        param_data[15] = map2odom_location[0];  // [map2odom] location_x
        param_data[16] = map2odom_location[1];  // [map2odom] location_y

        param_data[17] = 0.2f;                  // [Robot_Radius_Max]

        param_data[18] = 1.0f;                  // mindist_map's distance_range
        param_data[19] = 0.42f;                  // robot origin.x
        param_data[20] = 0.16f;                  // robot origin.y
        param_data[21] = 0.72f;                  // robot width
        param_data[22] = 0.32f;                  // robot height

    }else{
        ROS_INFO("******************************************************************");
        ROS_INFO("[xyi2_brain]: Params are unprepared, init with the default values.");
        ROS_INFO("******************************************************************");
        enable_param[0] = false;
        enable_param[1] = true;
        enable_param[2] = false;
        enable_param[3] = false;

        param_data[12] = 0;                     // [lsr] location_x
        param_data[13] = 0;                     // [lsr] location_y
        param_data[14] = 0.1;                   // [lsr] location_yaw

        param_data[15] = 0;                     // [map2odom] location_x
        param_data[16] = 0;                     // [map2odom] location_y

        param_data[17] = 0.2f;                  // [Robot_Radius_Max]

        param_data[18] = 1.0f;                  // mindist_map's distance_range
        param_data[19] = 0.25f;                  // robot origin.x
        param_data[20] = 0.17f;                  // robot origin.y
        param_data[21] = 0.5f;                  // robot width
        param_data[22] = 0.34f;                  // robot height
    }

    thread_manage->SLAM_ParamInit(workroute, enable_param, param_data);

    ros::spin();

    delete thread_manage;

    return 0;
}



