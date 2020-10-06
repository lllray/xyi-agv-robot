#include "tfm.h"

/* ********************************************************************************
 * author: chenyingbing
 * time: 20161019   09:38   in XIAMEN University
 * illustration:
 *      basic matrix function using eigen library
 *
 * *******************************************************************************/

// ==========================================================================================
// TFM: 载体到参考系
/*
 *  c(pitch)c(yaw)  s(pitch)s(roll)c(yaw)-c(roll)s(yaw)     s(pitch)c(roll)c(yaw)+s(roll)s(yaw)     px
 *  c(pitch)s(yaw)  s(pitch)s(roll)s(yaw)+c(roll)c(yaw)     s(pitch)c(roll)s(yaw)-s(roll)c(yaw)     py
 *  -s(pitch)       c(pitch)s(roll)                         c(pitch)c(roll)                         pz
 *  0               0                                       0                                       1.0
 *
 *  qw*qw + qx*qx - qy*qy - qz*qz   2*qx*qy - 2*qw*qz               2*qx*qz + 2*qw*qy               px
 *  2*qx*qy + 2*qw*qz               qw*qw - qx*qx + qy*qy - qz*qz   2*qy*qz - 2*qw*qx               py
 *  2*qx*qz - 2*qw*qy               2*qy*qz + 2*qw*qx               qw*qw - qx*qx - qy*qy + qz*qz   pz
 *  0                               0                               0                               1.0
 *
 *  c(yaw)  -s(yaw)     0     px
 *  s(yaw)  c(yaw)      0     py
 *  0       0           1     pz
 *  0       0           0     1.0
 *
 *  roll    = std::atan2(tmatrix.data()[6],tmatrix.data()[10]);	// A6/A10
    pitch   = std::asin(-tmatrix.data()[2]);                    // A2
    yaw     = std::atan2(tmatrix.data()[1],tmatrix.data()[0]);	// A1/A0

    static double p; p = PRY[0]*0.5;
    static double r; r = PRY[1]*0.5;
    static double y; y = PRY[2]*0.5;
    posedta.orientation.w = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);	//      When yaw: 	cos(yaw*0.5f)
    posedta.orientation.x = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);	//		0
    posedta.orientation.y = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);	//		0
    posedta.orientation.z = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);	//		sin(yaw*0.5f)
*/

static Matrix4d GeoPose2Matrix4dArray(geometry_msgs::Pose &posedta)
{
    double ArrayGet[16];
    double px; px = posedta.position.x;
    double py; py = posedta.position.y;
    double pz; pz = posedta.position.z;

    double qw; qw = posedta.orientation.w;
    double qx; qx = posedta.orientation.x;
    double qy; qy = posedta.orientation.y;
    double qz; qz = posedta.orientation.z;

    ArrayGet[0] = qw*qw + qx*qx - qy*qy - qz*qz;
    ArrayGet[1] = 2*qx*qy + 2*qw*qz;
    ArrayGet[2] = 2*qx*qz - 2*qw*qy;
    ArrayGet[3] = 0;

    ArrayGet[4] = 2*qx*qy - 2*qw*qz;
    ArrayGet[5] = qw*qw - qx*qx + qy*qy - qz*qz;
    ArrayGet[6] = 2*qy*qz + 2*qw*qx;
    ArrayGet[7] = 0;

    ArrayGet[8] = 2*qx*qz + 2*qw*qy;
    ArrayGet[9] = 2*qy*qz - 2*qw*qx;
    ArrayGet[10] = qw*qw - qx*qx - qy*qy + qz*qz;
    ArrayGet[11] = 0;

    ArrayGet[12] = px;
    ArrayGet[13] = py;
    ArrayGet[14] = pz;
    ArrayGet[15] = 1.0;

    Map<Matrix4d> MatrixReturn(ArrayGet);

    return MatrixReturn;
}

void TFM::active(void)
{
    isactive = true;
}

bool TFM::isActive(void)
{
    return isactive;
}

void TFM::fill(Matrix4d &dta_in)
{
    boost::recursive_try_mutex::scoped_try_lock try_lock(mutex_try_fresh);
    tmatrix = dta_in;
}

Matrix4d TFM::returndata(void)
{
    boost::recursive_try_mutex::scoped_try_lock try_lock(mutex_try_fresh);
    return tmatrix;
}

// ================================================================================================================
// CTool_tfm
static bool have_renew = false;
void CTool_tfm::renew_data(geometry_msgs::Pose odom, float *lsrl)
{
    static bool init = true;

    odomdata = odom;
    std::memcpy(lsrlocation, lsrl, sizeof(float) * 3);

    static Eigen::Matrix4d mat1; mat1 = GeoPose2Matrix4dArray(odom);
    TFM_Odom2Baselink_R.fill(mat1);

    static RM_NORM mat2;
    if(init)
    {
        mat2.tmatrix.data()[12] = lsrl[0];
        mat2.tmatrix.data()[13] = lsrl[1];
        mat2.tmatrix.data()[14] = lsrl[2];

        TFM_Baselink2Laser_R.fill(mat2.tmatrix);
    }

    static Eigen::Matrix4d mat3;
    mat3 = mat1 * mat2.tmatrix;
    TFM_Odom2Laser_R.fill(mat3);

    if(init)
    {
        TFM_Odom2Baselink_R.active();
        TFM_Baselink2Laser_R.active();
        TFM_Odom2Laser_R.active();
        init = false;
    }

    have_renew = true;
}

void CTool_tfm::tf_brocasterFun(void)
{
    if(have_renew)
    {
        static bool init_first = true;

        /*
        //Parent: map_r, Child: odom_r
        static tf::TransformBroadcaster tfbr;
        static tf::Transform tftrans;

        tftrans.setOrigin(tf::Vector3(0,
                                       0,
                                        0));
        tftrans.setRotation(tf::Quaternion(0,
                                            0,
                                             0,
                                              1));

        tfbr.sendTransform(tf::StampedTransform(tftrans, ros::Time::now(), "/map_r", "/odom_r"));*/


        //Parent: odom_r, Child: base_link_r
        static tf::TransformBroadcaster tfbr0;
        static tf::Transform tftrans0;

        tftrans0.setOrigin(tf::Vector3(odomdata.position.x,
                                        odomdata.position.y,
                                         0));
        tftrans0.setRotation(tf::Quaternion(odomdata.orientation.x,
                                             odomdata.orientation.y,
                                              odomdata.orientation.z,
                                               odomdata.orientation.w));

        tfbr0.sendTransform(tf::StampedTransform(tftrans0, ros::Time::now(), "/odom_r", "/base_link_r"));

        //Parent: base_link_r, Child: base_laser_link_r
        static tf::TransformBroadcaster tfbr1;
        static tf::Transform tftrans1;

        if(init_first)
        {
            tftrans1.setOrigin(tf::Vector3(lsrlocation[0],
                                           lsrlocation[1],
                                            lsrlocation[2]));
            tftrans1.setRotation(tf::Quaternion(0,
                                                 0,
                                                  0,
                                                   1));
        }

        tfbr1.sendTransform(tf::StampedTransform(tftrans1, ros::Time::now(), "/base_link_r", "/base_laser_link_r"));

        /* insteaded by /Robot_Port_Scan_S.(/base_laser_link_r)
        //Parent: base_laser_link_r, Child: base_laser_link
        static tf::TransformBroadcaster tfbr2;
        static tf::Transform tftrans2;

        if(init_first)
        {
            tftrans2.setOrigin(tf::Vector3(0,
                                           0,
                                           0));
            tftrans2.setRotation(tf::Quaternion(0,
                                                 0,
                                                  0,
                                                   1));
        }

        tfbr2.sendTransform(tf::StampedTransform(tftrans2, ros::Time::now(), "/base_laser_link_r", "/base_laser_link"));
        */

        init_first = false;
    }
}

void CTool_tfm::tf_brocaster_show(geometry_msgs::Pose &s_orien)
{
    //Parent: odom_r, Child: orientation_link_r
    static tf::TransformBroadcaster tfbr0;
    static tf::Transform tftrans0;

    tftrans0.setOrigin(tf::Vector3(0,
                                    0,
                                     0));
    tftrans0.setRotation(tf::Quaternion(s_orien.orientation.x,
                                         s_orien.orientation.y,
                                          s_orien.orientation.z,
                                           s_orien.orientation.w));

    tfbr0.sendTransform(tf::StampedTransform(tftrans0, ros::Time::now(), "/odom_r", "/orientation_link_r"));
}


