#ifndef __unit_table_H
#define __unit_table_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    typedef class table_mes
    {
    public:
        table_mes()
        {
            isfilled = false;

        }

        ~table_mes()
        {

        }

        // Graph check used.
        bool isfilled;
        int node_id;

    }table_mes;

    typedef class unit_table
    {
    public:
        unit_table()
        {
            grids.clear();
        }

        ~unit_table()
        {

        }

        int size;
        int size_xy;

        int length;
        int width;
        int height;

        std::vector<table_mes> grids;

    private:


    }unit_table;



#endif
