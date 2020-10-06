/* ********************************************************************************
 * author: chenyingbing
 * time: 20171026   16:27   in XIAMEN University
 * illustration:
 *      mange the navigation task.
 *
 * *******************************************************************************/

#include "navi_manage.h"
#include "../map/struct_map.h"
#include "gpath_plan/asearch.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// param of diff_globalpath()
#define diff_range              2.0f
#define diff_seek_step          1.0f
#define diff_disisclosed        0.1f

#define dis_change2nextnode     2.5f

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////
/// Port Func

void navi_manage::fill_goalpose(geometry_msgs::Pose _goalpose)
{
    if( (goalpose.position.x != _goalpose.position.x) ||
        (goalpose.position.y != _goalpose.position.y)
      )
    {
        goalpose = _goalpose;
        _GeoPose2Matrix4dArray(goalpose, matpose_goal);

        flag_newgpose = true;
    }
}

void navi_manage::fill_location(Eigen::Matrix4d &_matpose_robot, Eigen::Matrix4d &_matpose_m2lsr)
{
    matpose_robot = _matpose_robot;
    matpose_m2lsr = _matpose_m2lsr;
}

//////////////////////////////////////////////////

void navi_manage::run_gpath_planning(PARAM_ROBOT &srv_param_robot,
                                     Struct_Map &srv_map)
{
    static bool sta_asearch;

    if(flag_newgpose)
    {
        has_path_task = false;
        has_focusgoal = false;
        hasreachonce_goal = false;

        flag_gpath_show = false;

        srv_asearch.asearch_init(srv_param_robot.Robot_Radius_Max,
                                 srv_map.lreso[asearch_map_lvl], srv_map.lwidth[asearch_map_lvl], srv_map.lheight[asearch_map_lvl], srv_map.lsize[asearch_map_lvl],
                                 srv_map.map_xiuz[asearch_map_lvl]);

        sta_asearch = srv_asearch.asearch_search(srv_map.mdismap[asearch_map_lvl], srv_map.lmap[asearch_map_lvl],
                                                 matpose_robot, matpose_goal);

        if(sta_asearch)
            if(srv_asearch.asearch_extractpath(globalpath_data))
            {
                diff_globalpath();
            }else
                ROS_ERROR("[NAVI]: FAIL FIND PATH");

    }

    flag_newgpose = false;
}

void navi_manage::globalpath_show(void)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub = nstru.advertise<geometry_msgs::PolygonStamped>("path_show",1);

    if(flag_gpath_show)
    {
        gpath_show.header.stamp = ros::Time::now();
        gpath_show.header.frame_id = "/map_r";

        datapub.publish(gpath_show);
    }
}

//////////////////////////////////////////////////

void navi_manage::local_map_fresh(beam_lsrdta &_blsrdta,
                                  Struct_Map &srv_map)
{
    if(_blsrdta.isfull)
    {
        srv_local_obsmap.obsmap_init(srv_map.lreso[localobsmap_lvl], srv_map.lwidth[localobsmap_lvl], srv_map.lheight[localobsmap_lvl], srv_map.lsize[localobsmap_lvl],
                                     srv_map.map_xiuz[localobsmap_lvl], srv_map.mdismap_range);

        srv_local_obsmap.localmap_fresh(matpose_m2lsr, _blsrdta);

        flag_localmap_show = true;
    }

    _blsrdta.isfull = false;
}

void navi_manage::localmap_show(void)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("localmap",1);

    if(flag_localmap_show)
    {
        srv_local_obsmap.mdismap_show.header.frame_id = "/map_r";
        srv_local_obsmap.mdismap_show.header.stamp = ros::Time::now();

        datapub.publish(srv_local_obsmap.mdismap_show);
    }
}

//////////////////////////////////////////////////

void navi_manage::stop_control(void)
{
    srv_controller.stop_control();
}

void navi_manage::controlloop(float control_T, PARAM_ROBOT &srv_param_robot,
                              Struct_Map &srv_map, beam_lsrdta &_blsrdta)
{
    #define task1_T     1.0f    // T = 1.0s, f = 1hz.
    #define task2_T     0.1f    // T = 0.1s, f = 10hz.
    #define task3_T     0.05f   // T = 0.05s, f = 20hz
    static int task1_ci_T = round(task1_T / control_T), task1_ci = 0;
    static int task2_ci_T = round(task2_T / control_T), task2_ci = 0;
    static int task3_ci_T = round(task3_T / control_T), task3_ci = 0;

    if(task1_ci == 0)
        longterm_controlloop(task1_T, srv_map, _blsrdta);

    if(task2_ci == 0)
        shortterm_controlloop(task2_T, srv_param_robot, srv_map);

    if(task3_ci == 0)
        rtime_controlloop(task3_T);

    ++task1_ci, ++task2_ci, ++task3_ci;
    if(task1_ci >= task1_ci_T) task1_ci = 0;
    if(task2_ci >= task2_ci_T) task2_ci = 0;
    if(task3_ci >= task3_ci_T) task3_ci = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////
/// Private Func

// Func: refine globalpath_data.
void navi_manage::diff_globalpath(void)
{
    static int add_j, _size;
    static Eigen::Vector2f dta_get;
    static float dis;

    static std::vector<Eigen::Vector2f> tmppath_data;

    static int j;

    // globalpath_data: [goal] .... [start]
    _size = globalpath_data.size() - 1;

    globalpath_data[0](0) = matpose_goal.data()[12];        // modify to exact goal.
    globalpath_data[0](1) = matpose_goal.data()[13];

    add_j = std::ceil(diff_seek_step / srv_asearch.return_map_reso());

    tmppath_data.clear();
    tmppath_data.push_back(globalpath_data[0]);

    for(j = 0; j < _size ; j+=add_j)
    {
        dta_get = globalpath_data[j] - tmppath_data.back();
        dis = std::sqrt(dta_get(0)*dta_get(0) + dta_get(1)*dta_get(1));

        if(dis >= diff_range)
            tmppath_data.push_back(globalpath_data[j]);
    }

    dta_get = globalpath_data[_size] - tmppath_data.back();
    dis = std::sqrt(dta_get(0)*dta_get(0) + dta_get(1)*dta_get(1));

    if(dis < diff_disisclosed)
        tmppath_data.pop_back();                            // remove clostest point.

    // repack globalpath_data.
    globalpath_data.clear();

    for(j=0; j< tmppath_data.size(); j++)
    {
        globalpath_data.push_back(tmppath_data[j]);
    }

    has_path_task = true;

    // //////////////////////////////////////////
    // get gpath_show
    static geometry_msgs::Point32 point_get;

    gpath_show.polygon.points.clear();
    for(j=0; j<globalpath_data.size(); j++)
    {
        point_get.x = globalpath_data[j](0);
        point_get.y = globalpath_data[j](1);
        point_get.z = 0;

        gpath_show.polygon.points.push_back(point_get);
    }

    for(j=(globalpath_data.size()-1); j>=0; j--)
    {
        point_get.x = globalpath_data[j](0);
        point_get.y = globalpath_data[j](1);
        point_get.z = 0;

        gpath_show.polygon.points.push_back(point_get);
    }

    flag_gpath_show = true;

}

//////////////////////////////////////////////////

void navi_manage::show_robot_shape(void)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub = nstru.advertise<geometry_msgs::PolygonStamped>("robot_shape_show",1);

    static int i;
    static Eigen::Vector2d rcorner[4];
    static geometry_msgs::PolygonStamped robot_shape;
    static geometry_msgs::Point32 point_get;

    srv_dwindow.get_robot_shape(rcorner);

    robot_shape.polygon.points.clear();
    for(i=0; i<4; i++)
    {
        point_get.x = rcorner[i](0);
        point_get.y = rcorner[i](1);
        point_get.z = 0;

        robot_shape.polygon.points.push_back(point_get);
    }

    robot_shape.header.stamp = ros::Time::now();
    robot_shape.header.frame_id = "/map_r";
    datapub.publish(robot_shape);

}

// Func: get the focus_goal. set has_focusgoal = true.
void navi_manage::longterm_controlloop(float control_T,
                                       Struct_Map &srv_map, beam_lsrdta &_blsrdta)
{
    #define Radian2Degree           57.3f
    #define dis_see_add             1.0f
    static int i;
    static float dis;
    static Eigen::Vector2f v2frobot, v2fdeal;

    static Eigen::Vector2f v2_d;
    static Eigen::Vector3d v3_d;
    static rmat rmat_d;
    static Eigen::Matrix4d mat_d;
    static int yaw_degree_d, yaw_degree_d2;
    static bool has_seenode1;

    #define silimar_direction       15
    static Eigen::Vector2f v2_c1;
    static float dis_c1;
    static int direct_c1;

    if(has_path_task)
    {
        has_see_focusgoal = false;

        v2frobot(0) = matpose_robot.data()[12],
         v2frobot(1) = matpose_robot.data()[13];

        if(globalpath_data.size() > 0)
        {
            // get the closest node whose distance to the robot is more than dis_change2nextnode.
            for(i=0; i<globalpath_data.size(); i++)
            {
                v2fdeal = globalpath_data[i] - v2frobot;
                dis = std::sqrt(v2fdeal(0)*v2fdeal(0) + v2fdeal(1)*v2fdeal(1));

                if(dis < dis_change2nextnode)
                    break;
            }

            if(i>0)
                globalpath_data.erase(globalpath_data.begin()+i, globalpath_data.end());

            isfinal_goal = (globalpath_data.size() == 1);

            // select beteewn the following nodes.
            // node[1] = v2_c1, node[2] = v2_c2, ..., node[final] = goal.
            has_seenode1 = false;

            /// to following node[0].
            v2_d = globalpath_data.back();                                                  v2_c1 = v2_d;
            rmat_d.mat(12) = v2_d(0), rmat_d.mat(13) = v2_d(1);

            mat_d = matpose_robot.inverse() * rmat_d.mat;
            _Matrix4dGetXYYaw(mat_d, v3_d);

            dis = std::sqrt(mat_d(12)*mat_d(12) + mat_d(13)*mat_d(13)) + dis_see_add;       dis_c1 = dis;
            yaw_degree_d = std::floor(v3_d(2) * Radian2Degree) + _blsrdta.beam_size_2;      direct_c1 = yaw_degree_d;
            yaw_degree_d2 = yaw_degree_d + 1;

            has_seenode1 = (_blsrdta.beam_range[yaw_degree_d] > dis);
            has_see_focusgoal = has_seenode1;

            if(yaw_degree_d2 < _blsrdta.beam_size)
                has_seenode1 &= (_blsrdta.beam_range[yaw_degree_d2] > dis);

            focus_goal = v2_c1;

        }

        if(has_see_focusgoal == false)
        {

        }
        has_focusgoal = true;

        /// data show.
        static bool focusgoal_init_once = true;
        static ros::NodeHandle nstru;
        static ros::Publisher datapub = nstru.advertise<geometry_msgs::PoseStamped>("focus_goal",1);
        static geometry_msgs::PoseStamped focusgoal_get;

        if(focusgoal_init_once)
        {
            focusgoal_get.header.frame_id = "/map_r";

            float p = -0.75f;
            focusgoal_get.pose.orientation.w = cos(p);
            focusgoal_get.pose.orientation.x = 0;
            focusgoal_get.pose.orientation.y = sin(p);
            focusgoal_get.pose.orientation.z = 0;
        }

        focusgoal_get.header.stamp = ros::Time::now();
        focusgoal_get.pose.position.x = focus_goal(0);
        focusgoal_get.pose.position.y = focus_goal(1);

        datapub.publish(focusgoal_get);
    }

    // ROS_INFO("LONGTERM_CONTROL");
}

//////////////////////////////////////////////////

// Func: according to the focus_goal, control the robot based on the locapnavigation method.
void navi_manage::shortterm_controlloop(float control_T,
                                        PARAM_ROBOT &param_robot, Struct_Map &srv_map)
{
    #define task_T     0.4f
    static int task_ci_T = round(task_T / control_T), task_ci = 0;
    static bool ifkeep_ostate;

    /// Period: control_T
    // localpath algorithm.
    srv_dwindow.fresh_state(param_robot, matpose_robot,
                            srv_map, srv_local_obsmap,
                            srv_controller.state_v, srv_controller.state_gyro);

    if(has_focusgoal && (hasreachonce_goal == false))
    {
        ifkeep_ostate = srv_dwindow.if_keepstate();

        /// Period: task_T
        // change path in a lower frequency: 1.0f/task_T
        if((ifkeep_ostate == false) || (task_ci == 0))
        {
            task_ci = task_ci_T;

            srv_dwindow.search_pathes(task_T,
                                      focus_goal, vgyro_target_get);

            ifkeep_ostate = true;
        }
        ++task_ci;
        if(task_ci >= task_ci_T) task_ci = 0;

    }else
        vgyro_target_get(0, 0);

    show_robot_shape();

    // ROS_INFO("SHORTTERM_CONTROL");
}

void navi_manage::rtime_controlloop(float control_T)
{
    static float d1, d2, dis;
    static const Eigen::Vector2f vgyro_zero(0, 0);

    if( vgyro_target_get == vgyro_zero)
        srv_controller.stop_control();
    else
    {
        if(isfinal_goal == true)
        {
            d1 = matpose_goal(12) - matpose_robot(12);
            d2 = matpose_goal(13) - matpose_robot(13);
            dis = std::sqrt(d1*d1 + d2*d2);

            if(dis < dis_on_reachingmode)
                if(dis < dis_hasreached_goal)
                    hasreachonce_goal = true;
                else
                {
                    vgyro_target_get(0) = dis;
                    vgyro_target_get(1) = std::atan2(d1, d2);
                }
        }

        if(hasreachonce_goal)
        {
            d1 = _Matrix4dGetYaw(matpose_goal);
            d2 = _Matrix4dGetYaw(matpose_robot);

            vgyro_target_get(0) = 0;
            vgyro_target_get(1) =  d1 - d2;

            if(vgyro_target_get(1) > NAVI_PI) vgyro_target_get(1) -= NAVI_DOUBLE_PI;
            if(vgyro_target_get(1) < -NAVI_PI) vgyro_target_get(1) += NAVI_DOUBLE_PI;

        }

        // acelelimit_control
        srv_controller.acelelimit_control(control_T, vgyro_target_get);
        // srv_controller.instant_control(vgyro_target_get);
    }
}



