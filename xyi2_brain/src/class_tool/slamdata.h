#ifndef __slamdata_H
#define __slamdata_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "tf/transform_broadcaster.h"
    #include "tf/transform_listener.h"

    #include "tfm.h"

    typedef class SLAM_DATA
    {
    public:
        SLAM_DATA()
        {
            #define Dmatrix Msta_m2odom.data()
            Dmatrix[0] = 1,     Dmatrix[4] = 0;     Dmatrix[8] = 0,     Dmatrix[12] = 0,
            Dmatrix[1] = 0,     Dmatrix[5] = 1;     Dmatrix[9] = 0,     Dmatrix[13] = 0,
            Dmatrix[2] = 0,     Dmatrix[6] = 0,     Dmatrix[10] = 1,    Dmatrix[14] = 0,
            Dmatrix[3] = 0,     Dmatrix[7] = 0,     Dmatrix[11] = 0,    Dmatrix[15] = 1;

            /*
            Dmatrix[0] = 0.9397,    Dmatrix[4] = -0.34202;     Dmatrix[8] = 0,     Dmatrix[12] = 0.1,
            Dmatrix[1] = 0.34202,   Dmatrix[5] = 0.9397;   Dmatrix[9] = 0,     Dmatrix[13] = 0.3,
            Dmatrix[2] = 0,         Dmatrix[6] = 0,     Dmatrix[10] = 1.0,  Dmatrix[14] = 0,
            Dmatrix[3] = 0,         Dmatrix[7] = 0,     Dmatrix[11] = 0,    Dmatrix[15] = 1.0;  */

            #define Kmatrix Msta_emat.data()
            /*
            Kmatrix[0] = 0.9397,     Kmatrix[4] = -0.34202;     Kmatrix[8] = 0,     Kmatrix[12] = 0.4,
            Kmatrix[1] = 0.34202,    Kmatrix[5] = 0.9397;       Kmatrix[9] = 0,     Kmatrix[13] = 0,
            Kmatrix[2] = 0,          Kmatrix[6] = 0,            Kmatrix[10] = 1.0,  Kmatrix[14] = 0,
            Kmatrix[3] = 0,          Kmatrix[7] = 0,            Kmatrix[11] = 0,    Kmatrix[15] = 1.0;*/


            Kmatrix[0] = 1,     Kmatrix[4] = 0;     Kmatrix[8] = 0,     Kmatrix[12] = 0.4,
            Kmatrix[1] = 0,     Kmatrix[5] = 1;     Kmatrix[9] = 0,     Kmatrix[13] = -0.3,
            Kmatrix[2] = 0,     Kmatrix[6] = 0,     Kmatrix[10] = 1.0,  Kmatrix[14] = 0,
            Kmatrix[3] = 0,     Kmatrix[7] = 0,     Kmatrix[11] = 0,    Kmatrix[15] = 1.0;
        }

        ~SLAM_DATA()
        {

        }

        void Msta_m2odom_tfrefresh(bool enalble_tf,
                                   Eigen::Matrix4d &mat_o2lsr,
                                   Eigen::Matrix4d &mat_m2lsr);

        void Msta_m2odom_tfrefresh2(bool enalble_tf,
                                    Eigen::Matrix4d &mat_m2odom);

        Eigen::Matrix4d Msta_m2odom;        // map_exist use.

        Eigen::Matrix4d Msta_source2lsr;  // slam_exist use.
        Eigen::Matrix4d Msta_source2robot;  // slam_exist use.

    private:
        Eigen::Matrix4d Msta_emat;

        void Msta_m2odom_norm(void);

    }SLAM_DATA;



#endif
