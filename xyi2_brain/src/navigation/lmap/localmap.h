#ifndef __localmap_H
#define __localmap_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "../navi_config.h"

    #include "../../map/struct_map_config.h"

    #include <nav_msgs/OccupancyGrid.h>
    #include <geometry_msgs/Pose.h>

    #include "../../class_tool/hfunc.h"
    #include "../../matchings/struct_lsr_filter.h"

    typedef class hisdta_store
    {
    public:
        hisdta_store()
        {
            isfull = false;
            nodes.clear();
        }

        ~hisdta_store()
        {

        }

        bool isfull;
        std::vector<Eigen::Vector3i> nodes;

    private:

    }hisdta_store;

    typedef class local_obsmap
    {
    public:
        local_obsmap()
        {
            map_size_c = 0;
            has_init = false;

            mdismap.clear();

        }

        ~local_obsmap()
        {

        }

        void obsmap_init(float _map_reso,
                         int _map_width, int _map_height, int _map_size,
                         geometry_msgs::Pose _map_xiuz, float _mdismap_range);

        void localmap_fresh(Eigen::Matrix4d &matpose_m2lsr, beam_lsrdta &_blsrdta);



        std::vector<float> mdismap;
        nav_msgs::OccupancyGrid mdismap_show;

    private:
        float map_reso, inverse_map_reso;
        int map_width, map_height, map_size;
        geometry_msgs::Pose map_xiuz;

        float mdismap_range;

        float mdis_default;

        int map_size_c;
        bool has_init;

        void get_map_p3ilocation(Eigen::Matrix4d &matpose_map, Eigen::Vector3i &v3i_out);



    }local_obsmap;


#endif

