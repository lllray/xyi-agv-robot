#ifndef __navi_func_H
#define __navi_func_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include <geometry_msgs/Pose.h>

    typedef class rmat
    {
public:
    rmat()
    {
        mat(0) = 1.0f, mat(4) = 0.0f, mat(8) = 0.0f,  mat(12) = 0.0f,
        mat(1) = 0.0f, mat(5) = 1.0f, mat(9) = 0.0f,  mat(13) = 0.0f,
        mat(2) = 0.0f, mat(6) = 0.0f, mat(10) = 1.0f, mat(14) = 0.0f,
        mat(3) = 0.0f, mat(7) = 0.0f, mat(11) = 0.0f, mat(15) = 1.0f;
    }

    ~rmat(){}

    Eigen::Matrix4d mat;

private:

    }rmat;

    void _XyyawGetMatrix4d(Eigen::Vector3d &In_xyyaw, Eigen::Matrix4d &Out_MatrixDta);
    float _Matrix4dGetYaw(Eigen::Matrix4d &In_MatrixDta);
    void _Matrix4dGetXYYaw(Eigen::Matrix4d &In_MatrixDta, Eigen::Vector3d &Out_xyyaw);

    float _GeoPoseGetYaw(geometry_msgs::Pose &posedta);

    void _GeoPose2Matrix4dArray(geometry_msgs::Pose &posedta, Matrix4d &matrix_r);
    void _Matrix4dGetGeoPose(Matrix4d &MatrixDta, geometry_msgs::Pose *posedta);

    void _XyyawGetGeoPose(Eigen::Vector3d &In_xyyaw, geometry_msgs::Pose &posedta);

#endif
