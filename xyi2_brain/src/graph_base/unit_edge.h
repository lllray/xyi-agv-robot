#ifndef __unit_edge_H
#define __unit_edge_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #define edge_gcost_add  5

    enum type_edge{edge_normal = false, edge_loop = true};

    typedef class unit_edge
    {
    public:
        unit_edge()
        {
            edge_type = edge_normal;

            edge_gcost = 1;
        }

        ~unit_edge()
        {

        }

        /// basic data
        bool edge_type;

        int  edge_gcost;

        Eigen::Matrix3d mes_cov;
        Eigen::Matrix4d mes_data;

        /// adjacency list data
        int fnode_id;
        int tnode_id;

    }unit_edge;


#endif
