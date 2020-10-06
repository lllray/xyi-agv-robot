#include "slamdata.h"

/* ********************************************************************************
 * author: chenyingbing
 * time: 20161019   09:38   in XIAMEN University
 * illustration:
 *      basic slamdata caculation.
 *
 * *******************************************************************************/
static void Matrix4dGetGeoPose(Matrix4d &MatrixDta, geometry_msgs::Pose &posedta)
{
    static double PRY[3];
    PRY[1] = atan2(MatrixDta.data()[6],MatrixDta.data()[10]);	// A6/A10
    PRY[0] = asin(-MatrixDta.data()[2]);                        // A2
    PRY[2] = atan2(MatrixDta.data()[1],MatrixDta.data()[0]);	// A1/A0

    posedta.position.x = MatrixDta.data()[12];
    posedta.position.y = MatrixDta.data()[13];
    posedta.position.z = MatrixDta.data()[14];

    static double p; p = PRY[0]*0.5;
    static double r; r = PRY[1]*0.5;
    static double y; y = PRY[2]*0.5;
    posedta.orientation.w = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);	//      When yaw: 	cos(yaw*0.5f)
    posedta.orientation.x = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);	//		0
    posedta.orientation.y = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);	//		0
    posedta.orientation.z = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);	//		sin(yaw*0.5f)
}

void SLAM_DATA::Msta_m2odom_norm(void)
{
    static float norm;
    static float d1, d2;
    d1 = Msta_m2odom.data()[0];
    d2 = Msta_m2odom.data()[1];

    norm = std::sqrt(d1*d1 + d2*d2);

    d1 /= norm;
    d2 /= norm;

    Msta_m2odom.data()[0] = d1;
    Msta_m2odom.data()[5] = d1;
    Msta_m2odom.data()[1] = d2;
    Msta_m2odom.data()[4] = -d2;
}

/// >-----------------------------------------------------------------------------------
/// No.1
void SLAM_DATA::Msta_m2odom_tfrefresh(bool enalble_tf,
                                      Eigen::Matrix4d &mat_o2lsr,
                                      Eigen::Matrix4d &mat_m2lsr)
{
    static tf::TransformBroadcaster tfbr;
    static tf::Transform tftrans;
    static geometry_msgs::Pose posedata;

    Msta_m2odom = mat_m2lsr * mat_o2lsr.inverse();
    Msta_m2odom_norm();

    if(enalble_tf)
    {
        Matrix4dGetGeoPose(Msta_m2odom, posedata);

        /// Parent: map, Child: odom
        tftrans.setOrigin(tf::Vector3(posedata.position.x,
                                        posedata.position.y,
                                         posedata.position.z));
        tftrans.setRotation(tf::Quaternion(posedata.orientation.x,
                                             posedata.orientation.y,
                                              posedata.orientation.z,
                                               posedata.orientation.w));
        tfbr.sendTransform(tf::StampedTransform(tftrans, ros::Time::now(), "/map_r", "/odom_r"));
    }
}

/// >-----------------------------------------------------------------------------------
/// No.2
void SLAM_DATA::Msta_m2odom_tfrefresh2(bool enalble_tf,
                                       Eigen::Matrix4d &mat_m2odom)
{
    static tf::TransformBroadcaster tfbr;
    static tf::Transform tftrans;
    static geometry_msgs::Pose posedata;

    Msta_m2odom = mat_m2odom;
    Msta_m2odom_norm();

    if(enalble_tf)
    {
        Matrix4dGetGeoPose(Msta_m2odom, posedata);

        /// Parent: map, Child: odom
        tftrans.setOrigin(tf::Vector3(posedata.position.x,
                                        posedata.position.y,
                                         posedata.position.z));
        tftrans.setRotation(tf::Quaternion(posedata.orientation.x,
                                             posedata.orientation.y,
                                              posedata.orientation.z,
                                               posedata.orientation.w));
        tfbr.sendTransform(tf::StampedTransform(tftrans, ros::Time::now(), "/map_r", "/odom_r"));
    }
}
