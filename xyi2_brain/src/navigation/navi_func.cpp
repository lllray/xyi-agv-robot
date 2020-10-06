#include "navi_func.h"


void _XyyawGetMatrix4d(Eigen::Vector3d &In_xyyaw, Eigen::Matrix4d &Out_MatrixDta)
{
    float d_cos, d_sin;

    d_sin = std::sin(In_xyyaw(2));
    d_cos = std::cos(In_xyyaw(2));

    Out_MatrixDta.setZero();

    Out_MatrixDta.data()[0] = d_cos;    Out_MatrixDta.data()[4] = -d_sin;                                       Out_MatrixDta.data()[12] = In_xyyaw(0);
    Out_MatrixDta.data()[1] = d_sin;    Out_MatrixDta.data()[5] = d_cos;                                        Out_MatrixDta.data()[13] = In_xyyaw(1);
                                                                            Out_MatrixDta.data()[10] = 1.0f;
                                                                                                                Out_MatrixDta.data()[15] = 1.0f;
}

float _Matrix4dGetYaw(Eigen::Matrix4d &In_MatrixDta)
{
    return std::atan2(In_MatrixDta.data()[1],In_MatrixDta.data()[0]);
}

void _Matrix4dGetXYYaw(Eigen::Matrix4d &In_MatrixDta, Eigen::Vector3d &Out_xyyaw)
{
    Out_xyyaw(0) = In_MatrixDta.data()[12];
    Out_xyyaw(1) = In_MatrixDta.data()[13];
    Out_xyyaw(2) = std::atan2(In_MatrixDta.data()[1],In_MatrixDta.data()[0]);
}

float _GeoPoseGetYaw(geometry_msgs::Pose &posedta)
{
    float aw,ax,ay,az;
    aw = posedta.orientation.w;
    ax = posedta.orientation.x;
    ay = posedta.orientation.y;
    az = posedta.orientation.z;

    return std::atan2(2*(aw*az+ax*ay),(1-2*(ay*ay+az*az)));
}

/* ********************************************************************************
 * author: chenyingbing
 * time: 20161019   09:38   in XIAMEN University
 * illustration:
 *      basic matrix function using eigen library
 *
 * *******************************************************************************/
void _GeoPose2Matrix4dArray(geometry_msgs::Pose &posedta, Matrix4d &matrix_r)
{
    double qw = posedta.orientation.w;
    double qx = posedta.orientation.x;
    double qy = posedta.orientation.y;
    double qz = posedta.orientation.z;

    matrix_r.data()[0] = qw*qw + qx*qx - qy*qy - qz*qz;
    matrix_r.data()[1] = 2*qx*qy + 2*qw*qz;
    matrix_r.data()[2] = 2*qx*qz - 2*qw*qy;
    matrix_r.data()[3] = 0;

    matrix_r.data()[4] = 2*qx*qy - 2*qw*qz;
    matrix_r.data()[5] = qw*qw - qx*qx + qy*qy - qz*qz;
    matrix_r.data()[6] = 2*qy*qz + 2*qw*qx;
    matrix_r.data()[7] = 0;

    matrix_r.data()[8] = 2*qx*qz + 2*qw*qy;
    matrix_r.data()[9] = 2*qy*qz - 2*qw*qx;
    matrix_r.data()[10] = qw*qw - qx*qx - qy*qy + qz*qz;
    matrix_r.data()[11] = 0;

    matrix_r.data()[12] = posedta.position.x;
    matrix_r.data()[13] = posedta.position.y;
    matrix_r.data()[14] = posedta.position.z;
    matrix_r.data()[15] = 1.0;

}

/* ********************************************************************************
 * author: chenyingbing
 * time: 20161019   09:38   in XIAMEN University
 * illustration:
 *      basic matrix function using eigen library
 *
 * *******************************************************************************/
void _Matrix4dGetGeoPose(Matrix4d &MatrixDta, geometry_msgs::Pose *posedta)
{
    double PRY[3];
    PRY[1] = atan2(MatrixDta.data()[6],MatrixDta.data()[10]);	// A6/A10
    PRY[0] = asin(-MatrixDta.data()[2]);                        // A2
    PRY[2] = atan2(MatrixDta.data()[1],MatrixDta.data()[0]);	// A1/A0

    posedta->position.x = MatrixDta.data()[12];
    posedta->position.y = MatrixDta.data()[13];
    posedta->position.z = MatrixDta.data()[14];

    double p = PRY[0]*0.5;
    double r = PRY[1]*0.5;
    double y = PRY[2]*0.5;
    posedta->orientation.w = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);	//When yaw: 	cos(yaw*0.5f)
    posedta->orientation.x = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);	//		0
    posedta->orientation.y = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);	//		0
    posedta->orientation.z = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);	//		sin(yaw*0.5f)
}

/* ********************************************************************************
 * author: chenyingbing
 * time: 20171107   14:13   in XIAMEN University
 * illustration:
 *      basic matrix function using eigen library
 *
 * *******************************************************************************/
void _XyyawGetGeoPose(Eigen::Vector3d &In_xyyaw, geometry_msgs::Pose &posedta)
{
    float y = In_xyyaw(2) * 0.5f;

    posedta.position.x = In_xyyaw(0);
    posedta.position.y = In_xyyaw(1);
    posedta.position.z = 0;

    posedta.orientation.w = cos(y); //When yaw: 	cos(yaw*0.5f)
    posedta.orientation.x = 0;
    posedta.orientation.y = 0;
    posedta.orientation.z = sin(y); //              sin(yaw*0.5f)

}



