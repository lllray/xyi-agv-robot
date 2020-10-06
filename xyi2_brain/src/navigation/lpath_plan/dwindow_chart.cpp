/* ********************************************************************************
 * author: chenyingbing
 * time: 20171102   14:07   in XIAMEN University
 * illustration:
 *      dynamic window method.
 *
 * *******************************************************************************/

#include "dwindow.h"

static int v_gyro_chart_size;
static std::vector<Eigen::Vector2f> dta_v_gyro_chart;

static void generate_v_gyro_chart(void)
{
    static bool has_init = false;
    if(has_init == false)
    {
        float sample_interval_v    = 0.05f;
        float sample_interval_gyro = 0.1745f;

        int i, j;
        int v_ic = std::ceil( (MAX_VELO - 0.0f) / sample_interval_v );
        int gyro_ic = std::ceil( (MAX_GYRO * 2) / sample_interval_gyro );

        sample_interval_v = (MAX_VELO - 0.0f) / v_ic;
        sample_interval_gyro = (MAX_GYRO * 2) / gyro_ic;

        Eigen::Vector2f v_gryo_zero(0, 0);
        Eigen::Vector2f get_v_gyro;
        dta_v_gyro_chart.clear();

        for(i=0; i<=v_ic; i++)
        {
            for(j=0; j<=gyro_ic; j++)
            {
                get_v_gyro(0) = i * sample_interval_v + 0.0f;
                get_v_gyro(1) = j * sample_interval_gyro + (-MAX_GYRO);

                if(get_v_gyro != v_gryo_zero)
                    dta_v_gyro_chart.push_back(get_v_gyro);

                /*
                cout << "[" << get_v_gyro(0) << ", "
                            << get_v_gyro(1) << "], ";  */
            }
        }
        // cout << endl << "aaa" << endl;

        v_gyro_chart_size = dta_v_gyro_chart.size();

        has_init = true;
    }
}


void dwindow::search_pathes(float _task_T,
                            Eigen::Vector2f _focus_goal, Eigen::Vector2f &_vgyro_get)
{
    #define min_safe_dis   0.01f

    /// **********************************************************************
    /// Get infomation.
    static rmat mat_goal;
    static Eigen::Matrix4d mat_r2goal;
    static Eigen::Vector3d v3d_map2goal, v3d_r2goal;

    mat_goal.mat(12) = _focus_goal(0), mat_goal.mat(13) = _focus_goal(1);
    mat_r2goal = mat_robot->inverse() * mat_goal.mat;
    _Matrix4dGetXYYaw(mat_r2goal, v3d_r2goal);

    _Matrix4dGetXYYaw(mat_goal.mat, v3d_map2goal);

    generate_v_gyro_chart();

    static double time_s;
    static boost::timer t_caculate; t_caculate.restart();

    /// **********************************************************************
    /// Srocess: get search space
    #define generate_path_time_default     3.0f

    static int id;

    static std::vector<Eigen::Vector2f> cord_v_gyros;
    static Eigen::Vector2f get_v_gyro;

    static std::vector<Eigen::Vector3d> gpath;
    static std::vector<Eigen::Vector4f> cord_values;
    static Eigen::Vector4f values_get, values_norm;

    cord_v_gyros.clear();
    cord_values.clear();
    values_norm = Eigen::Vector4f(0, 0, 0, 0);

    for(id=0; id<v_gyro_chart_size; id++)
    {
        get_v_gyro(0) = dta_v_gyro_chart[id](0);
        get_v_gyro(1) = dta_v_gyro_chart[id](1);

        {
            static float min_safevalue, f_safevalue, f_dir2goal, f_dis2goal, v_gyro_value;

            generate_path(generate_path_time_default, get_v_gyro(0), get_v_gyro(1), gpath);

            if( get_path_info(gpath, v3d_map2goal,
                              min_safevalue, f_safevalue,
                              f_dis2goal, f_dir2goal) )
            {
                /// abandon the unsafe path.
                if(min_safevalue <= min_safe_dis) continue;

                /// calculate the values.
                // min_safevalue
                if(min_safevalue >= Disnode_Unkown)
                    min_safevalue = 0.0f;
                else
                    min_safevalue = srv_map->mdismap_range - min_safevalue;
                if(min_safevalue < 0) min_safevalue = 0;

                if(f_safevalue >= Disnode_Unkown)
                    f_safevalue = 0.0f;
                else
                    f_safevalue = srv_map->mdismap_range - f_safevalue;
                if(f_safevalue < 0) f_safevalue = 0;

                min_safevalue = 0.85f * min_safevalue + 0.15f * f_safevalue;

                // f_dir2goal

                // f_dis2goal

                // v_gyro_value
                v_gyro_value = 0.25f * (std::abs(get_v_gyro(0) - state_v) +
                                        std::abs(get_v_gyro(1) - state_gyro)) +
                               0.75f * (MAX_VELO - get_v_gyro(0)) ;

                values_get = Eigen::Vector4f(min_safevalue, f_dir2goal, f_dis2goal, v_gyro_value);

                cord_v_gyros.push_back(get_v_gyro);
                cord_values.push_back(values_get);

                values_norm += values_get;
            }

            /*
            for(int j=0; j<gpath.size(); j++)
            {
                cout << "[" << gpath[j](0) << ", "
                            << gpath[j](1) << ", "
                            << gpath[j](2) << "], ";
            }
            */
        }
    }

    /// **********************************************************************
    /// Srocess Value System.

    // Eigen::Vector4f(min_safevalue, f_dir2goal, f_dis2goal, v_gyro_value)
    static Eigen::Vector4f k_values;
    static float fvalues_c_min, fvalue;

    k_values = Eigen::Vector4f(0.1f, 0.05f, 0.75f, 0.1f);

    static int i;
    static int _size;
    static Eigen::Vector2f best_v_gyro;

    has_selected_path = false;

    _size = cord_v_gyros.size();
    best_v_gyro = Eigen::Vector2f(0, 0);

    if(_size > 0)
    {
        for(i=0; i<4; i++)
            values_norm(i) = 1.0f / values_norm(i);

        fvalues_c_min = INFINITY;
        for(i=0; i<_size; i++)
        {
            fvalue = k_values(0) * cord_values[i](0) * values_norm(0) +
                      k_values(1) * cord_values[i](1) * values_norm(1) +
                       k_values(2) * cord_values[i](2) * values_norm(2) +
                        k_values(3) * cord_values[i](3) * values_norm(3);

            if(fvalue < fvalues_c_min)
            {
                fvalues_c_min = fvalue;
                best_v_gyro = cord_v_gyros[i];

                has_selected_path = true;
            }
        }
    }

    if(has_selected_path == false)
    {
        // best_v_gyro(1) = state_gyro >= 0 ? MAX_GYRO : -MAX_GYRO;
        ROS_INFO("[NAVI MES] protecting. ");
    }
    else{
        ROS_INFO("[NAVI MES] %f, %f.", best_v_gyro(0), best_v_gyro(1));
    }

    _vgyro_get = best_v_gyro;

    show_selected_path(generate_path_time_default, best_v_gyro);

    // time_s = t_caculate.elapsed() * 1000;
    // ROS_INFO("[search_pathes]<%d>: %.1lf ms", _has_see_focusgoal, time_s);  // >> sample use 50~80 ms
}

void dwindow::show_selected_path(float _generate_path_time, Eigen::Vector2f &_v_gyro)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub = nstru.advertise<nav_msgs::Path>("dwindow_getpath",1);
    static nav_msgs::Path show_path;
    static geometry_msgs::PoseStamped pose_get;

    static int i;
    static std::vector<Eigen::Vector3d> spath;

    show_path.header.frame_id = "/map_r";
    show_path.header.stamp = ros::Time::now();

    pose_get.header = show_path.header;
    show_path.poses.clear();

    if(has_selected_path)
    {
        generate_path(_generate_path_time, _v_gyro(0), _v_gyro(1), spath);

        for(i=0; i<spath.size(); i++)
        {
            _XyyawGetGeoPose(spath[i], pose_get.pose);
            pose_get.pose.position.z = 0.1f;

            show_path.poses.push_back(pose_get);
        }

        datapub.publish(show_path);
    }
    /*
    else
        ROS_ERROR("DWINDOW PATH NUM == 0");*/
}
