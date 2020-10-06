#include "model_laser.h"

/* ********************************************************************************
 * author: chenyingbing
 * time: 20170313 21:25 - 20170317   in XIAMEN University
 * illustration:
 *      use a kind of three level tree to construct the map.
 *
 * *******************************************************************************/

// -------------------------------------------------------------------------------
void Model_Laser::Lsr_ParamInit(float *param)
{
    z_hit = param[0];
    z_rand = param[1];

    param_wait = true;
}

bool Model_Laser::Lsr_ModelInit(sensor_msgs::LaserScanConstPtr& lsr_in)
{
    if(!Lsr_model_sta)
    {
        c_rand = 1.0f/lsr_in->range_max;
        zc_rand = z_rand * c_rand;

        if (param_wait)
        {
            angle_range[0]  = lsr_in->angle_min;
            angle_range[1]  = lsr_in->angle_max;
            range_max       = lsr_in->range_max;
            angle_increment = lsr_in->angle_increment;
            angle_increment_2 = angle_increment * 0.5f;
            lsr_size = lsr_in->ranges.size();

            range_max_95percent = range_max * 0.95f;

            Lsr_model_sta = true;
        }
    }

    return Lsr_model_sta;
}

// -------------------------------------------------------------------------------
float Model_Laser::Lsr_Model_Gauss(float z)
{
    return (z_hit * z + zc_rand);
}

/*
float Model_Laser::Lsr_Model_Gauss(THTree_Map& map_service, char Lvlid, Eigen::Vector2i Ixy)
{
    int lid;
    float z;
    #define ix  Ixy.data()[0]
    #define iy  Ixy.data()[1]
    Ixy;

    if ( ((ix>=0)&&(ix<map_service.level_width[Lvlid])) &&
         ((iy>=0)&&(iy<map_service.level_height[Lvlid]))
       )
    {
        // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
        // std::exp(-(z * z) * gauss_d)
        //      0 x sigama = 1
        //      1 x sigama = 0.6065
        //      2 x sigama = 0.1353
        //      3 x sigama = 0.011
        //z = map_service.NNmapdata[lid];
        // pro_lsr[i].pro = z_hit * std::exp(-(z * z) * gauss_d) + z_rand * c_rand;

        lid = iy * map_service.level_width[Lvlid] + ix;
        z = map_service.Nmapdata_lv[Lvlid][lid].mdist;

        return (z_hit * z + z_rand * c_rand);
    }

    return 0.0f;
}
*/






