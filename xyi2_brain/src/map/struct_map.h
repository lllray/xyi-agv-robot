#ifndef __struct_map_H
#define __struct_map_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "geometry_msgs/Pose.h"
    #include "std_msgs/String.h"
    #include "nav_msgs/OccupancyGrid.h"

    #include "sensor_msgs/LaserScan.h"

    #include "../class_tool/slamdata.h"
    #include "../class_tool/tfm.h"
    #include "../sensors/model_laser.h"

    #include "struct_map_config.h"

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    #define Lvl_Num         2
    #define Lvl_LowReso     0
    #define Lvl_HighReso    1

    typedef class Struct_Map
    {
    public:
        Struct_Map()
        {
            param_wait = true;

            /// >-------------------------------------------------------------------------------
            /// Common Part
            lmap_sta = false;
            nmap_sta = false;
            mdismap_sta = false;

            lreso[0] = 0.1f;
            lreso[1] = 0.05f;

            pow_lreso[0] = lreso[0] * lreso[0];
            pow_lreso[1] = lreso[1] * lreso[1];

            lwidth[0] = 2048;
            lwidth[1] = 4096;

            lheight[0] = 2048;
            lheight[1] = 4096;

            map_size[0] = lreso[0] * lwidth[0];
            map_size[1] = lreso[0] * lheight[0];

            L0_Expand = 2.0f;
            L1_Expand = 1.0f;   /// fixed

            _L0_Expand = 1.0f / L0_Expand;
            _L1_Expand = 1.0f / L1_Expand;

            mdismap_range = 2.0f;

            map_isalloc = false;
            Map_Alloc_Init();

        }

        ~Struct_Map()
        {

        }

        /// >-------------------------------------------------------------------------------
        /// Common Part
        void Map_PramInit(float *param, float _mdismap_range);
        bool Map_alloc_sta(void);

        float sigama;
        float map_size[2];

        float L0_Expand;
        float L1_Expand;
        unsigned int lwidth[2], lheight[2], lsize[2];
        float lreso[2], pow_lreso[2];

        geometry_msgs::Pose map_xiuz[2];
        geometry_msgs::Pose map_origin[2];

        /// >-------------------------------------------------------------------------------
        /// MFunc_Mapexisting
        bool lmap_sta;
        bool nmap_sta;
        bool mdismap_sta;

        float mdismap_range;

        vector<float> lnmap[2];                 //   mapping map.
        nav_msgs::OccupancyGrid lmap[2];        //   grid map.
        vector<float> mdismap[2];               //   distance_min map.

        bool SlamInit_ExistMap(const nav_msgs::OccupancyGridConstPtr &map_s);
        void LMap_Pubcheck(char lvl);
        void LMap_UpdateNMap(Model_Laser &srv_lsrmodel);
        void NMap_Pubcheck(char lvl);
        void Mdismap_Pubcheck(char lvl);

        void SMap_Pubcheck(void);

    private:
        /// >-------------------------------------------------------------------------------
        /// Common Part
        bool param_wait;
        float powlreso_nigama[2];
        float Nmap_sigama;

        bool map_isalloc;
        void Map_Alloc_Init(void);

        float _L0_Expand;
        float _L1_Expand;


    }Struct_Map;

#endif
