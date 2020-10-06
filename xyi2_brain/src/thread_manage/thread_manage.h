#ifndef __thread_manage_H
#define __thread_manage_H

    #include "../pkg_include.h"

    #include "../class_tool/param_robot.h"
    #include "../class_tool/slamdata.h"
    #include "../class_tool/tfm.h"
    #include "../class_tool/hfunc.h"

    #include "../map/struct_map.h"
    #include "../sensors/model_laser.h"
    #include "../matchings/al_match.h"
    #include "../matchings/struct_lsr_filter.h"
    #include "../graph_base/struct_graph.h"

    #include "../com_bus/scom/scom_manage.h"

    #include "../navigation/navi_manage.h"

    typedef class Thread_Manage
    {
    public:
        Thread_Manage()
        {
            // >--------------------------------------------------------------
            // Goal_Thread
            goalpose_sub = nh_.subscribe("/move_base_simple/goal", 1, &Thread_Manage::task_sub_goalpose, this);

            trun_task_navigating = boost::bind(&Thread_Manage::TASK_NAVIGATING,this);
            thread_task_navigating = new boost::thread(trun_task_navigating);

            // >--------------------------------------------------------------
            // Odom_Thread
            odom_sub = nh_.subscribe("/RobotPort_Odom", 1, &Thread_Manage::task_sub_odom, this);
            todom_sub = nh_.subscribe("/base_pose_ground_truth", 1, &Thread_Manage::task_sub_todom, this);

            // >--------------------------------------------------------------
            // Manage_Thread

            trun_task_managemain = boost::bind(&Thread_Manage::TASK_MANAGE_MAIN,this);
            thread_task_managemain = new boost::thread(trun_task_managemain);

            // >--------------------------------------------------------------
            // Map_s_Thread
            map_s_sub = nh_.subscribe("/RobotPort_Map_s", 1, &Thread_Manage::task_sub_map_s, this);

            // >--------------------------------------------------------------
            // Lsr_Thread
            lsr_sub = nh_.subscribe("/RobotPort_Scan", 1, &Thread_Manage::task_sub_lsr, this);

            trun_task_lsrget = boost::bind(&Thread_Manage::TASK_LSR,this);
            thread_task_lsrget = new boost::thread(trun_task_lsrget);

            // >--------------------------------------------------------------
            // Scom_Thread
            cmdvel_sub = nh_.subscribe(Topics_Cmd, 1, &Thread_Manage::task_sub_cmdvel, this);

            beep_sub = nh_.subscribe("RobotPort_BeepAck", 1, &Thread_Manage::task_sub_beepcmd, this);

            trun_task_scom_receive = boost::bind(&Thread_Manage::TASK_SCOM_RDATA, this);
            thread_task_scom_receive = new boost::thread(trun_task_scom_receive);

            // >--------------------------------------------------------------
            // GraphOpt_Thread
            button_graphopt_sub = nh_.subscribe("/RobotPort_Button_GraphOpt", 1, &Thread_Manage::task_sub_button_graphopt, this);
        }

        ~Thread_Manage()
        {
            srv_scom.SCOMClose();

            thread_task_managemain->interrupt();
            thread_task_lsrget->interrupt();
            thread_task_navigating->interrupt();
            thread_task_graphopt->interrupt();
            thread_task_scom_receive->interrupt();

            delete thread_task_managemain;
            delete thread_task_lsrget;
            delete thread_task_navigating;
            delete thread_task_graphopt;
            delete thread_task_scom_receive;
        }

        void SLAM_ParamInit(std::string &workspace_route, bool *enable, float *param);

    private:
        ros::NodeHandle nh_;

        CTool_tfm    tool_tfm;
        PARAM_ROBOT  param_robot;
        SLAM_DATA    srv_packdata;
        ScomThread_Manage srv_scom;
        list_graph   srv_graph;
        Struct_Map   srv_map;
        Model_Laser  srv_lsrmodel;
        Al_Match     srv_almatch;
        navi_manage  srv_navigation;

        float param_lsrlocation[3];

        // >--------------------------------------------------------------
        // Goal_Thread
        ros::Subscriber goalpose_sub;
        void task_sub_goalpose(const geometry_msgs::PoseStamped& gposedta);

        void TASK_ONEOFF_GLOBALPATH(void);

        boost::thread *thread_task_navigating;
        boost::function0<void> trun_task_navigating;
        void TASK_NAVIGATING(void);

        // >--------------------------------------------------------------
        // Manage_Thread
        ros::Subscriber odom_sub, todom_sub;
        void task_sub_odom(const nav_msgs::Odometry& odomdta);
        void task_sub_todom(const nav_msgs::Odometry& todomdta);

        boost::thread *thread_task_managemain;
        boost::function0<void> trun_task_managemain;
        void TASK_MANAGE_MAIN(void);

        // >--------------------------------------------------------------
        // Map_s_Thread
        ros::Subscriber map_s_sub;
        void task_sub_map_s(const nav_msgs::OccupancyGridConstPtr& map_s);

        // >--------------------------------------------------------------
        // Lsr_Thread
        ros::Subscriber lsr_sub;
        void task_sub_lsr(const sensor_msgs::LaserScanConstPtr& lsr_s);

        boost::thread *thread_task_lsrget;
        boost::function0<void> trun_task_lsrget;
        void TASK_LSR(void);

        // >--------------------------------------------------------------
        // GraphOpt_Thread
        ros::Subscriber button_graphopt_sub;
        void task_sub_button_graphopt(const std_msgs::String& message);

        boost::thread *thread_task_graphopt;
        boost::function0<void> trun_task_graphopt;
        void TASK_GRAPH_OPT(void);

        // >--------------------------------------------------------------
        // Scom_Thread
        ros::Subscriber cmdvel_sub;
        void task_sub_cmdvel(const geometry_msgs::Twist& cmd_vel);

        ros::Subscriber beep_sub;
        void task_sub_beepcmd(const std_msgs::String& message);

        boost::thread *thread_task_scom_receive;
        boost::function0<void> trun_task_scom_receive;
        void TASK_SCOM_RDATA(void);

    }Thread_Manage;



#endif
