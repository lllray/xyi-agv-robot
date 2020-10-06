#ifndef __model_laser_H
#define __model_laser_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "sensor_msgs/LaserScan.h"

    typedef class Model_Laser
    {
    public:
        Model_Laser()
        {
            Lsr_model_sta = false;

            ANGLE_RANGE_LIMIT = 3.1415926575f; //3.1415926575f;// 1.570680628f;

            lsr_size = 0;

            param_wait = false;
            z_hit = 0.95f;
            z_rand = 0.05f;
            c_rand = 0.1f;

        }

        ~Model_Laser()
        {

        }

        float ANGLE_RANGE_LIMIT;

        int lsr_size;
        float range_max, range_max_95percent;
        float angle_range[2];
        float angle_increment;
        float angle_increment_2;

        void Lsr_ParamInit(float *param);

        bool Lsr_ModelInit(sensor_msgs::LaserScanConstPtr& lsr_in);

        float Lsr_Model_Gauss(float z);

    private:
        //-------------------------------------------------------------------------
        ///Lsr_Param_Init
        bool param_wait;
        float z_hit, z_rand;

        ///Lsr_Model_Init
        bool Lsr_model_sta;

        float c_rand;

        float zc_rand;


    }Model_Laser;
#endif






