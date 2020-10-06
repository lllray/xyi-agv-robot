#ifndef __struct_graph_H
#define __struct_graph_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "../map/struct_map.h"

    #include "../matchings/al_match_aidclass.h"
    #include "../matchings/struct_lsr_filter.h"

    #include "grapgh_base_config.h"

    #include "unit_kdtree.h"
    #include "unit_table.h"
    #include "unit_edge.h"
    #include "unit_node.h"

    enum s_fresh_akeynode{S_change2newknode = 0xFF,         // actions.
                          S_change2oldknode = 0xF0,
                          S_freshnode       = 0x01,
                          S_oldknode        = 0x00};

    typedef struct as_queue_node
    {
    public:
        /// messages for ranking.
        int node_id;
        int locate_id;
        float f_value;

        /// other messages.
        int parent_id;
        int edge_cid;
        float g_value;
        float h_value;

    }as_queue_node;

    typedef class list_graph
    {
    public:
        list_graph()
        {
            hasinit = false;

            graph_nodes.clear();

            enable_addnew = true;

            table_map_init = false;
        }

        ~list_graph()
        {
            for(int i=0; i<graph_nodes.size(); i++)
            {
                graph_nodes[i].free_node();
            }

            graph_nodes.clear();
        }

        // [PORT data] ********************************************************************************

        std::vector<unit_node> graph_nodes;
        int akey_node;
        int akey_node_old;

        // [PORT function] ****************************************************************************
        void graph_init(Struct_Map *srv_map,
                        m_lsrdta *lsr_filter,
                        float sx_interval, float sy_interval, float syaw_interval);

        unsigned char slam_fresh_akeynode(bool enable_addlsr,
                                          Struct_Map *srv_map,
                                          m_lsrdta *mlsr_in,
                                          Eigen::Matrix4d &mat_kf2lsr,
                                          Eigen::Matrix4d &mat_source2lsr,
                                          MATCH_PERFORMANCE &perform_match);

        void disable_addnew(void);
        void graph_pack_data(std::string &catkin_route);

        void slaminteract_map_show(void);

    private:
        // [PORT] param *******************************************************************************

        bool hasinit;
        bool enable_addnew;

        int vertexNum;
        int edgeNum;

        Eigen::Vector2f map_origin;
        Eigen::Vector2f map_xiuz;

        float param_interval_x;
        float param_interval_x_2;

        float param_interval_y;
        float param_interval_y_2;

        float param_interval_yaw;
        float param_interval_yaw_2;

        // [data] unit_table *************************************************************************
        float table_inrange_x[2];
        float table_inrange_y[2];
        float table_inrange_yaw[2];

        unit_table graph_table;
        bool table_map_init;
        nav_msgs::OccupancyGrid table_map;      // show for rviz

        int get_table_locateid(bool &is_inrange, Eigen::Matrix4d &posi_map, Eigen::Vector3i &ixy_r);

        // [data] unit_kdtree ***********************************************************************


        // graph function ***************************************************************************
        // struct_graph.cpp
        void graph_topics_pub(void);


    }list_graph;


#endif
