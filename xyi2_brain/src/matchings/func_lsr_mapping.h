#ifndef __func_lsr_mapping_H
#define __func_lsr_mapping_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"

    #include "geometry_msgs/Pose.h"
    #include "std_msgs/String.h"
    #include "nav_msgs/OccupancyGrid.h"
    using namespace Eigen;

    #include "../class_tool/tfm.h"
    #include "../class_tool/slamdata.h"
    #include "../map/struct_map.h"
    #include "../sensors/model_laser.h"

    #include "struct_localmapping_chart.h"
    #include "struct_lsr_filter.h"

    #define mapg_slam_bondary   100
    enum type_match{tpye_normal=0, type_continuous=2};
    enum type_match_stategy{match_normal_hmap = 0+tpye_normal,                      /// mapping stategy when map exsits
                            match_continuous_hmp = 0+type_continuous,
                            match_normal_slam = mapg_slam_bondary+tpye_normal,      /// mapping stategy when slam mode.
                            match_continuous_slam = mapg_slam_bondary+type_continuous
                                                                             };

    typedef float (*func_lsrmatch)(char lvlmap_mapexist,
                                   char mode,
                                   Struct_Map &srv_map,
                                   Eigen::Matrix4d &dmat_kf2lsr,
                                   m_lmmapchart *srv_lmmapchart,
                                   bool isfull,
                                   int lsr_size,
                                   vector<vtype_lsr>::iterator &it_lsrdta);

    float mtype_lsrmatch(char lvlmap_mapexist,
                         char mode,
                         Struct_Map &srv_map,
                         Eigen::Matrix4d &dmat_kf2lsr,
                         m_lmmapchart *srv_lmmapchart,
                         bool isfull,
                         int lsr_size,
                         vector<vtype_lsr>::iterator &it_lsrdta);



#endif




