/* ********************************************************************************
 * author: chenyingbing
 * time: 20170605   15:19   in XIAMEN University
 * illustration:
 *      basic matrix function using eigen library
 *
 * *******************************************************************************/

#include "tfm.h"

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

void CTool_tfm::tf_slam_brocasterFun(Eigen::Matrix4d &mat_source2kf,
                                     Eigen::Matrix4d &mat_kf2lsr)
{
    static Eigen::Matrix4d mat_deal;
    static geometry_msgs::Pose pose_deal;

    //Parent: base_laser_link_r, Child: kframe_r
    static tf::TransformBroadcaster tfbr0;
    static tf::Transform tftrans0;

    mat_deal = mat_kf2lsr.inverse();
    Matrix4dGetGeoPose(mat_deal, pose_deal);

    tftrans0.setOrigin(tf::Vector3(pose_deal.position.x,
                                    pose_deal.position.y,
                                     0));
    tftrans0.setRotation(tf::Quaternion(pose_deal.orientation.x,
                                         pose_deal.orientation.y,
                                          pose_deal.orientation.z,
                                           pose_deal.orientation.w));

    tfbr0.sendTransform(tf::StampedTransform(tftrans0, ros::Time::now(), "/base_laser_link_r", "/kframe_r"));

    //Parent: kframe_r, Child: map_r
    static tf::TransformBroadcaster tfbr1;
    static tf::Transform tftrans1;

    mat_deal = mat_source2kf.inverse();
    Matrix4dGetGeoPose(mat_deal, pose_deal);

    tftrans1.setOrigin(tf::Vector3(pose_deal.position.x,
                                    pose_deal.position.y,
                                     0));
    tftrans1.setRotation(tf::Quaternion(pose_deal.orientation.x,
                                         pose_deal.orientation.y,
                                          pose_deal.orientation.z,
                                           pose_deal.orientation.w));

    tfbr1.sendTransform(tf::StampedTransform(tftrans1, ros::Time::now(), "/kframe_r", "/map_r"));
}


