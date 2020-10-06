#ifndef __tfm_H
#define __tfm_H

    #include <iostream>
    using namespace std;

    #include "std_msgs/String.h"

    #include <algorithm>
    #include <signal.h>
    #include <vector>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "tf/transform_broadcaster.h"
    #include "tf/transform_listener.h"

    // ==========================================================================================
    // RM
    typedef class RM_NORM
    {
    public:
        RM_NORM()
        {
            double *dta;
            dta = tmatrix.data();
            for(int i=0; i<16; i++) dta[i] = 0;

            dta[0] = 1.0; dta[5] = 1.0; dta[10] = 1.0;
            dta[12] = 0; dta[13] = 0; dta[14] = 0; dta[15] = 1.0;

        }
        ~RM_NORM(){}

        // values
        Matrix4d tmatrix;

    private:

    }RM_NORM;

    // TFM
    typedef class TFM
    {
    public:
        TFM()
        {
            isactive = false;

            double *dta;
            dta = tmatrix.data();
            for(int i=0; i<16; i++) dta[i] = 0;
            dta[0] = 1.0; dta[5] = 1.0; dta[10] = 1.0;
            dta[12] = 0; dta[13] = 0; dta[14] = 0; dta[15] = 1.0;

        }
        ~TFM(){}

        void fill(Matrix4d &dta_in);
        Matrix4d returndata(void);

        void active(void);
        bool isActive(void);
    private:
        bool isactive;

        Matrix4d tmatrix;

        boost::recursive_try_mutex mutex_try_fresh;
    }TFM;

    // ==========================================================================================
    // fixsize struct
    typedef struct CTool_tfm
    {
    public:
        CTool_tfm()
        {


        }

        ~CTool_tfm()
        {

        }

        TFM TFM_Odom2Laser_R;
        TFM TFM_Odom2Baselink_R;
        TFM TFM_Baselink2Laser_R;

        // tfm.cpp
        void renew_data(geometry_msgs::Pose odom, float *lsrl);
        void tf_brocasterFun(void);

        // tfm_slam.cpp
        void tf_slam_brocasterFun(Eigen::Matrix4d &mat_source2kf,
                                  Eigen::Matrix4d &mat_kf2lsr);

    private:
        float lsrlocation[3];
        geometry_msgs::Pose odomdata;

        // tfm.cpp
        void tf_brocaster_show(geometry_msgs::Pose &s_orien);

        // tfm_slam.cpp

    }CTool_tfm;

#endif
