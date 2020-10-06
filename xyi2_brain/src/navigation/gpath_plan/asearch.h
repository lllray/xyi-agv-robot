#ifndef __asearch_H
#define __asearch_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "../../class_tool/param_robot.h"
    #include "../../map/struct_map.h"

    #include <geometry_msgs/PolygonStamped.h>

    enum asearch_node_sta{ nsta_openlist = 1, nsta_closelist=2, nsta_unchecked=3};
    enum asearch_node_mode{ smode_neighbour4 = 4, smode_neighbour8 = 8};
    enum asearch_result{ asearch_r_wrongdes = 0, asearch_r_success = 1, asearch_r_runoff = 2, asearch_r_running = 3 };

    typedef class asearch_node
    {
    public:
        asearch_node()
        {
            node_sta = nsta_unchecked;
            p3i_parent = Eigen::Vector3i(-1, -1, -1);
            p3i_self = Eigen::Vector3i(-1, -1, -1);
            values_fghs = Eigen::Vector4f(0, 0, 0, 0);

        }

        ~asearch_node(){}

        asearch_node_sta node_sta;

        Eigen::Vector3i p3i_parent, p3i_self;  // (width_id, height_id, array_id)
        Eigen::Vector4f values_fghs;

    }asearch_node;

    typedef class asearch_manage
    {
    public:
        asearch_manage()
        {
            has_init = false;

            robot_minssafedis = 1.0f;

            map_size_c = 0;

            locate_node_active_now = -1;

            surlocate_nneighbours4.resize(4);
            surlocate_nneighbours8.resize(8);

            locate_openlist_fqueue.clear();
        }

        ~asearch_manage()
        {

        }

        void asearch_init(float _robot_minssafedis,
                          float _map_reso, int _map_width, int _map_height, int _map_size,
                          geometry_msgs::Pose _map_xiuz);

        bool asearch_search(std::vector<float> &mdismap, nav_msgs::OccupancyGrid &lmap,
                            Eigen::Matrix4d &mat_start, Eigen::Matrix4d &mat_goal);

        bool asearch_extractpath(std::vector<Eigen::Vector2f> &vpath);

        float return_map_reso(void);

    private:

        bool has_init;

        float robot_minssafedis;

        float map_reso,inverse_map_reso;
        int map_width, map_height, map_size;
        geometry_msgs::Pose map_xiuz;

        int map_size_c;
        std::vector<asearch_node> nodes_map;
        std::vector<int> locate_openlist_fqueue, locate_travelled_nodes;

        Eigen::Vector3i p3i_start, p3i_goal;    // (width_id, height_id, array_id)
        volatile int locate_node_active_now;

        float step_g;
        std::vector<int> surlocate_nneighbours4, surlocate_nneighbours8;

        void get_map_p3ilocation(Eigen::Matrix4d &matpose_map, Eigen::Vector3i &v3i_out);
        void get_map_location(Eigen::Vector3i &v3i_in, Eigen::Vector2f &v2f_out);

        void asearch_node_reset(int locate_node);

        float asearch_calculate_h(Eigen::Vector3i &checknode);
        float asearch_calculate_s(std::vector<float> &mdismap, Eigen::Vector3i &checknode);

        void asearch_add2fqueue(asearch_node &add_node);
        void asearch_fqueue_rerange(asearch_node &change_node);
        void asearch_add2openlist(char search_mode,
                                  std::vector<float> &mdismap, nav_msgs::OccupancyGrid &lmap);

        asearch_result asearch_if_exist(void);

        void asearch_set_start(std::vector<float> &mdismap);
        void asearch_set_node_active(void);

    }asearch_manage;


#endif

