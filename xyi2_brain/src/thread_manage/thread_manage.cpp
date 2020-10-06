#include "thread_manage.h"

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// > COMMON DATA

#define     Enable_ScomSend     true

/// >-----------------------------------------

static bool Enable_ActLsrThread = true;

static bool Enable_SLAM = false;
static bool Enable_Matching = false;
static bool Enable_RenewOdom = false;

static bool Enable_Scom = false;
static bool Once_RecevieOdom = false;

static bool Map_FirstInit = false;
static bool Manage_PramInit = false;

static bool Once_CalLocation = false;

static bool ACTIVE_CATKIN_ROUTE = false;
static std::string catkin_route, basic_route = "/src/xyi2_bringup/graphfiles";

/// >-----------------------------------------
static nav_msgs::OccupancyGridConstPtr taskptr_map;

static sensor_msgs::LaserScanConstPtr taskptr_lsr;
static beam_lsrdta taskdta_lsr;

/// >-----------------------------------------
boost::recursive_mutex mutex_lsrdta, mutex_lsrdta2;
boost::condition_variable_any cond_getlsr;                  // lsr data.
#define MUTEX_LOCK_LSRDTA   boost::recursive_mutex::scoped_lock lock(mutex_lsrdta);
#define MUTEX_LOCK_LSRDTA2  boost::recursive_mutex::scoped_lock lock(mutex_lsrdta2);

boost::recursive_mutex mutex_scom_pub, mutex_scom_sub;      // scom communication.
#define MUTEX_LOCK_SCOM_PUB     boost::recursive_mutex::scoped_lock lock(mutex_scom_pub);
#define MUTEX_LOCK_SCOM_SUB     boost::recursive_mutex::scoped_lock lock(mutex_scom_sub);

boost::recursive_mutex mutex_lmap;                          // lvl map.
#define MUTEX_LOCK_LMAP     boost::recursive_mutex::scoped_lock lock(mutex_lmap);

boost::recursive_mutex mutex_nmap;                          // mapping map.
#define MUTEX_LOCK_NMAP     boost::recursive_mutex::scoped_lock lock(mutex_nmap);

boost::recursive_mutex mutex_localmap;                      // local map.
#define MUTEX_LOCK_LOCALMAP     boost::recursive_mutex::scoped_lock lock(mutex_localmap);

boost::recursive_mutex mutex_graph;                         // graph data.
#define MUTEX_LOCK_GRAPH    boost::recursive_mutex::scoped_lock lock(mutex_graph);


// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >MAP_S THREAD

void Thread_Manage::task_sub_map_s(const nav_msgs::OccupancyGridConstPtr& map_s)
{
    static bool init_once = true;

    while(Manage_PramInit == false)
        boost::this_thread::sleep(boost::posix_time::milliseconds(25));

    if ( (Enable_SLAM == false) && init_once && Enable_Matching)
    {
        {   /// Fresh LMAP
            MUTEX_LOCK_LMAP

            while(srv_map.SlamInit_ExistMap(map_s) == false)
                    boost::this_thread::sleep(boost::posix_time::milliseconds(25));

            {   /// Use LMap to Update NMap
                MUTEX_LOCK_LMAP
                {
                    MUTEX_LOCK_NMAP
                    srv_map.LMap_UpdateNMap(srv_lsrmodel);
                }
            }

            {   /// Mapping to Correct TF_Map2Lsr, Once
                MUTEX_LOCK_NMAP
                srv_almatch.AlMatch_MappingPort(match_mapexist,
                                                tool_tfm,
                                                srv_packdata,
                                                srv_map,
                                                srv_lsrmodel,
                                                srv_graph,
                                                taskptr_lsr);
            }
        }

        Map_FirstInit = true;
        init_once = false;
    }
}

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >LSR THREAD

void Thread_Manage::task_sub_lsr(const sensor_msgs::LaserScanConstPtr& lsr_s)
{
    // for task i.
    if(taskptr_lsr == NULL)
    {
        MUTEX_LOCK_LSRDTA
        taskptr_lsr = lsr_s;
        srv_lsrmodel.Lsr_ModelInit(taskptr_lsr);

        cond_getlsr.notify_one();
    }
    else
    {
        if( taskptr_lsr.use_count() == 0)
        {
            MUTEX_LOCK_LSRDTA

            taskptr_lsr = lsr_s;
            srv_lsrmodel.Lsr_ModelInit(taskptr_lsr);

            cond_getlsr.notify_one();
        }
    }

    // for task ii.
    {
        MUTEX_LOCK_LSRDTA2

        sensor_msgs::LaserScanConstPtr taskptr_lsr_tmp = lsr_s;

        taskdta_lsr.reset();
        taskdta_lsr.fill(srv_lsrmodel, taskptr_lsr_tmp);

        taskptr_lsr_tmp.reset();
    }
}

void Thread_Manage::TASK_LSR(void)
{
    while(ros::ok())
    {
        try
        {
            boost::this_thread::interruption_point();
            ////////////////////////////////////////////////////////////////

            if(taskptr_lsr != NULL)
            {
                while(taskptr_lsr.use_count() == 0)
                {
                    {
                        MUTEX_LOCK_LSRDTA
                    }
                    cond_getlsr.wait(mutex_lsrdta);
                }

                // /////////////////////////////////////////////////////////

                if(Manage_PramInit && Enable_ActLsrThread)
                {
                    static bool RLSR_FLAG = true;
                    if(RLSR_FLAG)
                    {
                        ROS_INFO("Lsr Receive Once.");

                        RLSR_FLAG = false;
                    }

                    if(Enable_SLAM && (Map_FirstInit == false))
                    {
                        if(tool_tfm.TFM_Odom2Laser_R.isActive())
                        {
                            if( srv_almatch.AlMatch_MappingPort(match_slaminit,
                                                                tool_tfm,
                                                                srv_packdata,
                                                                srv_map,
                                                                srv_lsrmodel,
                                                                srv_graph,
                                                                taskptr_lsr))
                                Map_FirstInit = true;
                        }

                    }

                    else if(Map_FirstInit && Enable_Matching)
                    {
                        if(Enable_SLAM == false)
                        {   /// Mapping to Correct TF_Map2Lsr
                            MUTEX_LOCK_NMAP

                            srv_almatch.AlMatch_MappingPort(match_mapexist,
                                                            tool_tfm,
                                                            srv_packdata,
                                                            srv_map,
                                                            srv_lsrmodel,
                                                            srv_graph,
                                                            taskptr_lsr);
                        }

                        else
                        {   /// Mapping based on the keyframe in graph.

                            /*
                            cout << "0: " << param_robot.Robot_Width_Min << endl;
                            cout << "1: " << tool_tfm.TFM_Baselink2Laser_R.returndata().data()[12] << endl;
                            cout << "2: " << srv_packdata.Msta_m2odom.data()[12] << endl;
                            cout << "3: " << srv_map.L0_Expand << endl;
                            cout << "4: " << srv_lsrmodel.angle_increment_2 << endl;
                            cout << "5: " << srv_graph.graph_nodes.size() << endl;
                            cout << "6: " << taskptr_lsr->range_max << endl;
                            cout << "7: null" << endl;      */

                            srv_almatch.AlMatch_MappingPort(match_slammap,
                                                            tool_tfm,
                                                            srv_packdata,
                                                            srv_map,
                                                            srv_lsrmodel,
                                                            srv_graph,
                                                            taskptr_lsr);
                        }

                        Once_CalLocation = true;
                    }


                }

                // /////////////////////////////////////////////////////////
                taskptr_lsr.reset();
            }

            ////////////////////////////////////////////////////////////////
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        catch(boost::thread_interrupted&)
        {

        }
    }
}

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >GRAPHOPT_THREAD

void Thread_Manage::task_sub_button_graphopt(const std_msgs::String& message)
{
    static bool haveget_graph = false;

    if( Enable_SLAM && (haveget_graph == false) && (message.data == "graph_save") )
    {
        Enable_ActLsrThread = false;

        srv_graph.disable_addnew();

        trun_task_graphopt = boost::bind(&Thread_Manage::TASK_GRAPH_OPT,this);
        thread_task_graphopt = new boost::thread(trun_task_graphopt);


        haveget_graph = true;
    }
}

void Thread_Manage::TASK_GRAPH_OPT(void)
{
    if(ACTIVE_CATKIN_ROUTE)
    {
        MUTEX_LOCK_GRAPH

        srv_graph.graph_pack_data(catkin_route);
    }
    else
        ROS_ERROR(" TASK_GRAPH_OPT: ENABLE SLAM BUT NON ACCESSIBLE CATKIN_ROUTE.");
}

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >SCOM THREAD

void Thread_Manage::task_sub_cmdvel(const geometry_msgs::Twist& cmd_vel)
{
    if(Enable_Scom && Enable_ScomSend)
    {   MUTEX_LOCK_SCOM_PUB
        /// cmdvel
        // cout << "[ INFO ] Pub Once" << endl;
        srv_scom.SCOMPub_Cmdvel(cmd_vel);        // it works but is useless in this case.
    }
}

void Thread_Manage::task_sub_beepcmd(const std_msgs::String& message)
{
    if(Enable_Scom && Enable_ScomSend)
    {   MUTEX_LOCK_SCOM_PUB
        if(message.data == "beep_ack")
        {
            srv_scom.SCOMPub_BeepAck();
        }
    }

}

void Thread_Manage::TASK_SCOM_RDATA(void)
{
    geometry_msgs::Pose     DataOdom;
    geometry_msgs::Point32  DataPosi;
    geometry_msgs::Twist    DataTwist;

    geometry_msgs::Pose     SData_Orientation;

    while(ros::ok())
    {
        try
        {
            boost::this_thread::interruption_point();
            ////////////////////////////////////////////////////////////////

            if(Manage_PramInit && Enable_Scom)
            {   
                MUTEX_LOCK_SCOM_SUB
                if(srv_scom.SCOMSub(DataOdom,
                                    DataPosi,
                                    DataTwist,
                                    SData_Orientation))
                {
                    if(Enable_RenewOdom)
                    {
                        /// > DataOdom
                        tool_tfm.renew_data(DataOdom, param_lsrlocation);

                        /// > DataTwist-------------------------------------------------------------------------------

                        /// > Show orientation
                        // tool_tfm.tf_brocaster_show(SData_Orientation);

                        Once_RecevieOdom = true;
                    }
                }
            }

            ////////////////////////////////////////////////////////////////
            boost::this_thread::sleep(boost::posix_time::milliseconds(80));
        }
        catch(boost::thread_interrupted&)
        {

        }
    }
}

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >ODOM THREAD

void Thread_Manage::task_sub_todom(const nav_msgs::Odometry& todomdta)
{
    #define if_enable_show_true 0

    #if if_enable_show_true == 1
        cout << "[" <<  todomdta.pose.pose.position.x << ", "
                    <<  todomdta.pose.pose.position.y << ", "
                    <<  0 << "], ";
    #endif
}

void Thread_Manage::task_sub_odom(const nav_msgs::Odometry& odomdta)
{
    #define if_enable_position   0
    #define if_enable_orientation   1
    static geometry_msgs::Pose tfpub_odomdta;


    if(Manage_PramInit && Enable_RenewOdom && (Enable_Scom == false))
    {
        #if  if_enable_orientation == 1
            tfpub_odomdta.orientation = odomdta.pose.pose.orientation;
        #else
            tfpub_odomdta.orientation.w = 1.0f,
                    tfpub_odomdta.orientation.x = 0.0f, tfpub_odomdta.orientation.y = 0.0f, tfpub_odomdta.orientation.z = 0.0f;
        #endif

        #if  if_enable_position == 1
            tfpub_odomdta.position = odomdta.pose.pose.position;
        #endif

        tool_tfm.renew_data(tfpub_odomdta, param_lsrlocation);

        Once_RecevieOdom = true;
    }
}

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >GOAL THREAD

static volatile bool NAVI_STA_GPATHPLAN_BUSY = false;

void Thread_Manage::task_sub_goalpose(const geometry_msgs::PoseStamped& gposedta)
{

    if((Map_FirstInit) && (NAVI_STA_GPATHPLAN_BUSY == false))
    {
        NAVI_STA_GPATHPLAN_BUSY = true;

        srv_navigation.fill_goalpose(gposedta.pose);

        boost::thread thread_task_oneoff_globalpath(boost::bind(&Thread_Manage::TASK_ONEOFF_GLOBALPATH,this));

    }
    else
        ROS_ERROR("[WARNING]: fail to set goal, your operation is too quick!");

}

void Thread_Manage::TASK_ONEOFF_GLOBALPATH(void)
{
    srv_navigation.run_gpath_planning(param_robot,
                                      srv_map);

    NAVI_STA_GPATHPLAN_BUSY = false;
}

void Thread_Manage::TASK_NAVIGATING(void)
{
    #define control_T_milliseconds   50
    static const float control_T_s = control_T_milliseconds / 1000.0f;

    while(ros::ok())
    {
        try
        {
            boost::this_thread::interruption_point();
            ////////////////////////////////////////////////////////////////

            if( Once_CalLocation && (Enable_SLAM == false))
            {
                srv_navigation.fill_location(srv_packdata.Msta_source2robot,
                                             srv_packdata.Msta_source2lsr);

                {
                    MUTEX_LOCK_LSRDTA2
                    {
                        MUTEX_LOCK_LOCALMAP
                        srv_navigation.local_map_fresh(taskdta_lsr,
                                                       srv_map);
                    }
                }

                if(NAVI_STA_GPATHPLAN_BUSY == false)
                {
                    srv_navigation.controlloop(control_T_s, param_robot,
                                               srv_map, taskdta_lsr);
                }
                else
                    srv_navigation.stop_control();


            }

            ////////////////////////////////////////////////////////////////
            boost::this_thread::sleep(boost::posix_time::milliseconds(control_T_milliseconds));
        }
        catch(boost::thread_interrupted&)
        {

        }
    }
}

// **************************************************************************************
// >-------------------------------------------------------------------------------------
// >MANAGE THREAD

void Thread_Manage::SLAM_ParamInit(std::string &workspace_route, bool *enable, float *param)
{
    if(!Manage_PramInit)
    {
        Enable_SLAM = enable[0];
        Enable_Matching = enable[1];
        Enable_RenewOdom = enable[2];

        Enable_Scom = enable[3];

        if(Enable_SLAM)
        {
            catkin_route = workspace_route + basic_route;
            ACTIVE_CATKIN_ROUTE = true;
        }

        float param_data[9];

        param_data[0] = param[0];       // [lsr] sigama
        srv_map.Map_PramInit(param_data,
                             param[18]);

        param_data[0] = param[1];       // [lsr] z_hit
        param_data[1] = param[2];       // [lsr] z_rand
        srv_lsrmodel.Lsr_ParamInit(param_data);

        param_data[0] = param[3];       // SWin_XYRange[0]
        param_data[1] = param[4];       // SWin_XYRange[1]
        param_data[2] = param[5];       // SWin_XYreso[0]
        param_data[3] = param[6];       // SWin_XYreso[1]
        param_data[4] = param[7];       // SWin_YawRange[0]
        param_data[5] = param[8];       // SWin_YawRange[1]
        param_data[6] = param[9];       // SWin_YawReso[0]
        param_data[7] = param[10];      // SWin_YawReso[1]
        param_data[8] = param[11];      // H_square_sigma
        srv_almatch.Almatch_PramInit(param_data);

        param_lsrlocation[0] = param[12];
        param_lsrlocation[1] = param[13];
        param_lsrlocation[2] = param[14];

        srv_packdata.Msta_m2odom.data()[12] = param[15];
        srv_packdata.Msta_m2odom.data()[13] = param[16];

        param_robot.Robot_Radius_Max = param[17];
        param_robot.Robot_Origin[0] = param[19];
        param_robot.Robot_Origin[1] = param[20];
        param_robot.Robot_Width     = param[21];
        param_robot.Robot_Height    = param[22];

        if(Enable_RenewOdom == false)
        {
            geometry_msgs::Pose tfpub_odomdta;
            tfpub_odomdta.position.x = 0,
             tfpub_odomdta.position.y = 0,
              tfpub_odomdta.position.z = 0;
            tfpub_odomdta.orientation.x = 0;
             tfpub_odomdta.orientation.y = 0;
              tfpub_odomdta.orientation.z = 0;
               tfpub_odomdta.orientation.w = 1.0;
            tool_tfm.renew_data(tfpub_odomdta, param_lsrlocation);

            Once_RecevieOdom = true;
        }

        sin_chart_init();

        if(Enable_Scom)
        {
            srv_scom.SCOMInit();
        }

        Manage_PramInit = true;
    }
}

void Thread_Manage::TASK_MANAGE_MAIN(void)
{
    while(ros::ok())
    {
        ////////////////////////////////////////////////////////////////

        if(Manage_PramInit)
        {
            if(Enable_Matching)
            {
                static char MapPubT = 0;
                ++MapPubT;
                if(MapPubT >= 10)
                {
                    {
                        MUTEX_LOCK_LMAP
                        // srv_map.LMap_Pubcheck(0);
                    }

                    if(Enable_SLAM == true)
                    {
                        srv_graph.slaminteract_map_show();
                    }
                    else{

                        {
                            MUTEX_LOCK_LOCALMAP
                            srv_navigation.localmap_show();
                        }

                        // srv_map.NMap_Pubcheck(1);
                        srv_map.Mdismap_Pubcheck(0);
                        srv_navigation.globalpath_show();
                    }

                    MapPubT = 0;
                }
            }

            {
                tool_tfm.tf_brocasterFun();
            }

            // debug part
            if(1)
            {

            }
        }

        ////////////////////////////////////////////////////////////////
        boost::this_thread::sleep(boost::posix_time::milliseconds(25));
    }

}




