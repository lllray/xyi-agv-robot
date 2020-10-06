#ifndef __navi_manage_H
#define __navi_manage_H

    #include <iostream>
    using namespace std;

    #include <ros/ros.h>
    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include <geometry_msgs/Pose.h>
    #include <geometry_msgs/PolygonStamped.h>

    #include "../class_tool/param_robot.h"
    #include "../class_tool/tfm.h"
    #include "../map/struct_map.h"
    #include "../matchings/struct_lsr_filter.h"

    #include "navi_config.h"

    #include "navi_func.h"

    #include "gpath_plan/asearch.h"
    #include "lmap/localmap.h"
    #include "lpath_plan/dwindow.h"
    #include "rcontrol/r_controller.h"

    typedef class navi_manage
    {
    public:
        navi_manage()
        {
            flag_newgpose = false;
            flag_gpath_show = false;
            flag_localmap_show = false;

            has_path_task = false;
            has_focusgoal = false;

            vgyro_target_get(0, 0);

            isfinal_goal = false;
            hasreachonce_goal = false;
        }

        ~navi_manage()
        {

        }

        ///Port func
        //>
        void fill_goalpose(geometry_msgs::Pose _goalpose);
        void fill_location(Eigen::Matrix4d &_matpose_robot, Eigen::Matrix4d &_matpose_m2lsr);

        //>
        void run_gpath_planning(PARAM_ROBOT &srv_param_robot,
                                Struct_Map &srv_map);
        void globalpath_show(void);

        //>
        void local_map_fresh(beam_lsrdta &_blsrdta,
                             Struct_Map &srv_map);

        void localmap_show(void);

        void stop_control(void);
        void controlloop(float control_T, PARAM_ROBOT &srv_param_robot,
                         Struct_Map &srv_map, beam_lsrdta &_blsrdta);

    private:       

        // *****************************************************************************************
        ////////////////////////////////////////////////
        bool flag_newgpose;
        geometry_msgs::Pose goalpose;                   // geometry_msgs::Pose_goal's pose
        Eigen::Matrix4d matpose_goal;                   // matrix_goal's pose
        Eigen::Matrix4d matpose_robot, matpose_m2lsr;   // matrix_robot's pose, matrix_m2lsr's pose

        asearch_manage srv_asearch;

        bool flag_gpath_show;
        bool has_path_task;
        std::vector<Eigen::Vector2f> globalpath_data;
        geometry_msgs::PolygonStamped gpath_show;
        void diff_globalpath(void);

        ////////////////////////////////////////////////

        bool flag_localmap_show;
        local_obsmap   srv_local_obsmap;                // [port v] srv_local_obsmap.obsmap

        ////////////////////////////////////////////////

        bool has_focusgoal;
        bool has_see_focusgoal;
        Eigen::Vector2f focus_goal;

        dwindow srv_dwindow;

        Eigen::Vector2f vgyro_target_get;
        r_controller srv_controller;

        ////////////////////////////////////////////////

        void show_robot_shape(void);

        bool isfinal_goal;
        bool hasreachonce_goal;

        void longterm_controlloop(float control_T,
                                  Struct_Map &srv_map, beam_lsrdta &_blsrdta);

        void shortterm_controlloop(float control_T,
                                   PARAM_ROBOT &param_robot, Struct_Map &srv_map);

        void rtime_controlloop(float control_T);

    }navi_manage;


#endif
