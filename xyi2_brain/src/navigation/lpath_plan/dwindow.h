#ifndef __dwindow_H
#define __dwindow_H

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

    #include "../navi_config.h"

    #include "../navi_func.h"

    #include "../../class_tool/param_robot.h"
    #include "../../map/struct_map.h"
    #include "../lmap/localmap.h"

    #include <nav_msgs/Path.h>

    #define dwindow_abosolutely_safe    255

    enum running_mode{ runmode_stop = 0, runmode_pureforward = 1,
                       runmode_purerotating = 2, runmode_runleft = 3, runmode_runright = 4};

    typedef class dwindow
    {
    public:
        dwindow()
        {
            has_selected_path = false;
        }
        ~dwindow()
        {

        }

        // dwindow.cpp
        void get_robot_shape(Eigen::Vector2d *_rcorner);

        void fresh_state(PARAM_ROBOT &_param_robot, Eigen::Matrix4d &_mat_robot,
                         Struct_Map &_srv_map, local_obsmap &_localmap,
                         float _state_v, float _state_gyro);

        bool if_keepstate(void);

        // dwindow_chart.cpp
        void search_pathes(float _task_T,
                           Eigen::Vector2f _focus_goal, Eigen::Vector2f &_vgyro_get);

    private:

        PARAM_ROBOT *param_robot;
        Eigen::Matrix4d *mat_robot;
        Struct_Map *srv_map;
        local_obsmap *localmap;
        float state_v, state_gyro;

        float map_reso, inverse_map_reso;
        Eigen::Vector2d map_xiuz;
        int map_width, map_height, map_size;

        Eigen::Vector3d v3d_robot;

        // dwindow.cpp
        float calculate_v3dpose_safevaule(Eigen::Matrix4d &_mat_robotpose);

        float get_path_safevalue(std::vector<Eigen::Vector3d> &_pathnodes, float &_percentage);
        bool get_path_info(std::vector<Eigen::Vector3d> &_pathnodes, Eigen::Vector3d &gnode,
                           float &_min_safevalue, float &_f_safevalue,
                           float &_f_dis2goal, float &_f_dir2goal);

        void generate_path(float limit_t, float _v, float _gyro,
                           std::vector<Eigen::Vector3d> &_pathnodes);

        // dwindow_chart.cpp
        bool has_selected_path;
        void show_selected_path(float _generate_path_time, Eigen::Vector2f &_v_gyro);

    }dwindow;

#endif
