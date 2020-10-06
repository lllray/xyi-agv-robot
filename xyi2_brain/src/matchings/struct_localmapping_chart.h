#ifndef __struct_localmapping_chart_H
#define __struct_localmapping_chart_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"

    using namespace Eigen;

    #include "../map/struct_map.h"
    #include "struct_lsr_filter.h"

    typedef class m_lmmapchart
    {
    public:
        m_lmmapchart()
        {
            hasalloc = false;

            nmap.clear();
            nmap_cord.clear();

            map_pub.data.clear();
        }

        ~m_lmmapchart()
        {

        }

        unsigned int llength, lsize;
        float lreso;

        Eigen::Vector2f map_origin;
        Eigen::Vector2f map_xiuz;

        int node_id;

        std::vector<float> nmap;
        std::vector<int> nmap_cord;

        void reset(void);

        void buildmap(unsigned int nid,
                      Struct_Map *srv_map,
                      mi_lsrdta *mi_lsrin);
        void BUILDMAP_TASK(mi_lsrdta *mi_lsrin);

        void nmap_pubcheck(void);

    private:
        float _lreso;

        bool hasalloc;

        void fresh_value_chart(void);

        nav_msgs::OccupancyGrid map_pub;


    }m_lmmapchart;

#endif



