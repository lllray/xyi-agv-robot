#ifndef __unit_kframe_H
#define __unit_kframe_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "grapgh_base_config.h"

    #include "../map/struct_map.h"
    #include "../matchings/struct_lsr_filter.h"
    #include "../matchings/struct_localmapping_chart.h"

    #include "unit_edge.h"

    typedef struct node_lsrmes
    {
        unsigned int lsr_lid;
        Eigen::Matrix4d mat;
    }node_lsrmes;

    typedef class unit_node
    {
    public:
        unit_node()
        {
            num_subunit = 0;
            num_addadd = 0;
            nummax_subunit = 10;

            edges.clear();

            nmap_obs_chart.clear();
            nmap.clear();

            lmmapchart_deal = NULL;
        }

        ~unit_node()
        {

        }

        /// basic data
        Eigen::Matrix4d mat_source2lsr;

        mi_lsrdta lsrdta;                       // the data to generate local mapping chart.

        unsigned int nummax_subunit;
        unsigned char num_addadd;
        unsigned int num_subunit;

        /// adjacency list data
        int node_id;
        int locate_id;
        Eigen::Vector3i locate_xy;

        std::vector<unit_edge> edges;

        void alloc_init(Struct_Map *srv_map, m_lsrdta *lsr_in);

        bool if_available(Eigen::Matrix4d &mat2start, Eigen::Matrix4d &mat2target);

        void add_lsrdata(Struct_Map *srv_map,
                         Eigen::Matrix4d &mat_kf2lsr, m_lsrdta *lsr_in);

        void add_lsrdata_v2(bool enable_fresh,
                            Struct_Map *srv_map,
                            Eigen::Matrix4d &mat_kf2lsr, m_lsrdta *lsr_in);

        bool add_lsrdata_v3(unit_node &base_node,
                            Struct_Map *srv_map,
                            Eigen::Matrix4d &mat_bnode2lsr, m_lsrdta *lsr_in);

        /// V.2ed add
        // message in lsrdta.
        std::vector<bool>  nmap_obs_chart;
        std::vector<float> nmap;

        /// V.2ed+ add : speed up.
        m_lmmapchart *lmmapchart_deal;

        void free_node(void)
        {
            nmap_obs_chart.clear();
            nmap.clear();

            delete lmmapchart_deal;
        }

    private:

    }unit_node;



#endif
