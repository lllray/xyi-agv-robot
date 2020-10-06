/* ********************************************************************************
 * author: chenyingbing
 * time: 20170623 19:25  in XIAMEN University
 * illustration:
 *
 *
 *
 * *******************************************************************************/

// Sys Part **********************************************/
#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <string>
using namespace std;

// C_Plus Standard Library *******************************/
#include <algorithm>
#include <signal.h>
#include <c++/5/vector>
#include <boost/timer.hpp>

#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/recursive_mutex.hpp"

// TF Part ***********************************************/
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

// Message Part ******************************************/
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

// Math Part *********************************************/
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
using namespace Eigen;

// File I/O *********************************************/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

#define PI          3.1415925f
#define DOUBLE_PI   6.283185f

static void XyyawGetMatrix4d(Eigen::Vector3d &In_xyyaw, Eigen::Matrix4d &Out_MatrixDta)
{
    static float d_cos, d_sin;

    d_sin = std::sin(In_xyyaw(2));
    d_cos = std::cos(In_xyyaw(2));

    Out_MatrixDta.setZero();

    Out_MatrixDta.data()[0] = d_cos;    Out_MatrixDta.data()[4] = -d_sin;                                       Out_MatrixDta.data()[12] = In_xyyaw(0);
    Out_MatrixDta.data()[1] = d_sin;    Out_MatrixDta.data()[5] = d_cos;                                        Out_MatrixDta.data()[13] = In_xyyaw(1);
                                                                            Out_MatrixDta.data()[10] = 1.0f;
                                                                                                                Out_MatrixDta.data()[15] = 1.0f;
}

static void Matrix4dGetXYYaw(Eigen::Matrix4d &In_MatrixDta, Eigen::Vector3d &Out_xyyaw)
{
    Out_xyyaw(0) = In_MatrixDta.data()[12];
    Out_xyyaw(1) = In_MatrixDta.data()[13];
    Out_xyyaw(2) = std::atan2(In_MatrixDta.data()[1],In_MatrixDta.data()[0]);
}


static void protect_range_yaw(Eigen::Vector3d &vector)
{
    if(vector(2) > PI)
        vector(2) -= DOUBLE_PI;
    else if(vector(2) < -PI)
        vector(2) += DOUBLE_PI;
}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xyi2_test");
    ros::NodeHandle nstru("~");

    double time_s;
    boost::timer t_caculate;

    ROS_INFO(" XYI2_TEST MAIN.");

    if(0)
    {
        int i;
        int vertex_size, edge_num;
        std::vector<Eigen::Vector3d> opt_vertex, opt_edge, opt_d_vertex;
        std::vector<Eigen::Matrix3d> edges_info;

        edges_info.resize(7);

        opt_vertex.push_back(Eigen::Vector3d(4.79912, 0.150726, 0.214058));
        opt_edge.push_back(Eigen::Vector3d(0.904755, -2.25232, -1.93788));
        edges_info[0]<<15.2452,0,0,0,30.3808,0,0,0,19.6634;

        opt_vertex.push_back(Eigen::Vector3d(6.16024, -1.85587, -1.72383));
        opt_edge.push_back(Eigen::Vector3d(1.29558, 0.667005, 0.441496));
        edges_info[1]<<27.8264,0,0,0,19.2273,0,0,0,21.1046;

        opt_vertex.push_back(Eigen::Vector3d(6.62097, -3.235, -1.28233));
        opt_edge.push_back(Eigen::Vector3d(0.743813, 0.0158347, 0.109139));
        edges_info[2]<<15.4092,0,0,0,14.9655,0,0,0,15.5715;

        opt_vertex.push_back(Eigen::Vector3d(6.84697, -3.94114, -1.17319));
        opt_edge.push_back(Eigen::Vector3d(0.879267, 1.53258, 2.43288));
        edges_info[3]<<19.6764,0,0,0,23.0556,0,0,0,59.0415;

        opt_vertex.push_back(Eigen::Vector3d(8.59383, -4.15756, 1.25969));
        opt_edge.push_back(Eigen::Vector3d(1.00274, -0.0380415, 0.0654696));
        edges_info[4]<<14.3604,0,0,0,17.7151,0,0,0,13.7493;

        opt_vertex.push_back(Eigen::Vector3d(8.93514, -3.21969, 1.32516));
        opt_edge.push_back(Eigen::Vector3d(1.89237, 0.405371, 0.647361));
        edges_info[5]<<20.8629,0,0,0,18.3003,0,0,0,6.87042;

        opt_vertex.push_back(Eigen::Vector3d(9.00173, -1.29641, 1.97252));
        opt_edge.push_back(Eigen::Vector3d(1.20855, 0.422001, 0.546597));
        edges_info[6]<<16.0233,0,0,0,9.1919,0,0,0,9.65365;

        opt_vertex.push_back(Eigen::Vector3d(8.15843, -0.331333, 2.52999));

        vertex_size = opt_vertex.size();
        edge_num = opt_edge.size();

        opt_d_vertex.resize(vertex_size, Eigen::Vector3d(0, 0, 0));

        ROS_INFO(" LOOP CORRECTION: PREPARATION STEP2 > SUCCESS.");

        /// solution to the nonlinear least square problem.
        static int j, k, k_size, k_size_sub1;
        static int iterates_t, iterates_add_t;
        static double F_values, F_values_o;
        static double f_value;
        static Eigen::Vector3d vector_d, vector_residual;

        static Eigen::Matrix3d Bmat, Imat, dImat;
        static double step_alpha, step_alpha_yaw;
        static Eigen::Vector3d hdm;

        static double cos_d, sin_d;
        static double Dx_cos_d, Dx_sin_d, Dy_cos_d, Dy_sin_d;
        static double dk, d1, d2, d3;

        F_values_o = 100;
        iterates_t = edge_num * 30, iterates_add_t = 1;

        Bmat.setZero(), Imat.setZero();
        Bmat.data()[0] = 20.0f, Bmat.data()[4] = 20.0f, Bmat.data()[8] = 50.0f;
        Imat.data()[0] = 1.0f, Imat.data()[4] = 1.0f, Imat.data()[8] = 1.5f;

        step_alpha = 1.0f, step_alpha_yaw = 1.0f;

        // check if exists ERROR. But it is impossible to occur.
        if(opt_vertex.size() != (edge_num + 1))
            ROS_ERROR("<func> ceres_path_optimize: amazing: num_vertex != num_edge + 1");
        else
            k_size = (edge_num + 1);
            k_size_sub1 = edge_num;

            while(iterates_t--)
            {
                F_values = 0;

                /// path edges.
                //this_node_id = node_id_s;

                // j: opt_edge;     edges_info;
                // k: opt_vertex;   opt_d_vertex;
                for(j = (edge_num-1), k = edge_num; j>=0; j--, k--)
                //for(j = 0, k = 1; j<=(edge_num-1); j++, k++)
                {
                    cos_d = std::cos(opt_vertex[k-1](2));
                    sin_d = std::sin(opt_vertex[k-1](2));

                    Dx_cos_d = opt_edge[j](0) * cos_d;
                    Dx_sin_d = opt_edge[j](0) * sin_d;
                    Dy_cos_d = opt_edge[j](1) * cos_d;
                    Dy_sin_d = opt_edge[j](1) * sin_d;

                    vector_d(0) = opt_vertex[k-1](0) + Dx_cos_d - Dy_sin_d;
                    vector_d(1) = opt_vertex[k-1](1) + Dx_sin_d + Dy_cos_d;
                    vector_d(2) = opt_vertex[k-1](2) + opt_edge[j](2);
                    protect_range_yaw(vector_d);

                    vector_residual = opt_vertex[k] - vector_d;
                    protect_range_yaw(vector_residual);

                    //cout << "res1: " << opt_vertex[k](0) << ", " << opt_vertex[k](1) << ", " << opt_vertex[k](2) << endl;
                    //cout << "res2: " << vector_d(0) << ", " << vector_d(1) << ", " << vector_d(2) << endl;

                    f_value = vector_residual.transpose() * edges_info[j] * vector_residual;
                    F_values = F_values + f_value;

                    dk = 1.0f;
                    d1 = edges_info[j].data()[0] * vector_residual(0);
                    d2 = edges_info[j].data()[4] * vector_residual(1);
                    d3 = edges_info[j].data()[8] * vector_residual(2);

                    dImat.data()[0] = Imat.data()[0] * step_alpha;
                    dImat.data()[4] = Imat.data()[4] * step_alpha;
                    dImat.data()[8] = Imat.data()[8] * step_alpha_yaw;
                    dImat = (Bmat + dImat).inverse();

                    //cout << "d: " << d1 << ", " << d2 << ", " << d3 << endl;

                    if(k != edge_num)
                    {
                        opt_d_vertex[k](0) -= d1 * dk,  // negative gradient direction.
                        opt_d_vertex[k](1) -= d2 * dk,
                        opt_d_vertex[k](2) -= d3 * dk;
                    }

                    if(k != 1)
                    {
                        opt_d_vertex[k-1](0) -= (-d1 * dk);
                        opt_d_vertex[k-1](1) -= (-d2 * dk);

                        opt_d_vertex[k-1](2) -= ( d1 * dk * ( -1 + Dx_sin_d + Dy_cos_d ) +
                                                  d2 * dk * ( -1 - Dx_cos_d + Dy_sin_d ) +
                                                  d3 * dk * ( -1 )
                                                );
                    }
                }

                // revise the factors.
                if( F_values < F_values_o)
                {
                    step_alpha = step_alpha * 0.95f;
                    step_alpha_yaw = step_alpha_yaw * 0.95f;

                    F_values_o = F_values;
                }else{
                    iterates_t -= iterates_add_t;
                    iterates_add_t += 1;
                    if(iterates_t < 0) break;

                    step_alpha = step_alpha * 2.5f;
                    step_alpha_yaw = step_alpha_yaw * 2.5f;
                }

                for(k=1; k<k_size_sub1; k++)
                {
                    dImat.data()[0] = Imat.data()[0] * step_alpha;
                    dImat.data()[4] = Imat.data()[4] * step_alpha;
                    dImat.data()[8] = Imat.data()[8] * step_alpha_yaw;

                    hdm = (Bmat + dImat).inverse() * opt_d_vertex[k];
                    opt_d_vertex[k].setZero();

                    opt_vertex[k] += hdm;
                    protect_range_yaw(opt_vertex[k]);

                }

                cout << "F: " << F_values << endl;
            }

        ROS_INFO(" LOOP CORRECTION ONCE: FValue %f.", F_values);

        /// final correction to data.
        cout << "[ INFO ] FPath: " << endl;
        for(k = 0; k<= edge_num; k++)
        {
            cout << opt_vertex[k](0) << ", " << opt_vertex[k](1) << ", " << opt_vertex[k](2) << endl;
        }

        time_s = t_caculate.elapsed();
        ROS_INFO(" LOOP CORRECTION FINISH: %lf s.", time_s);
    }

    {
        if(0)
        {
            std::string base, route, full_name;
            route = "/home/cyb7369299/My_Workspace/xyi_v2_ws";
            base = "/src/xyi2_bringup/graphfiles/xyi_graph.data";

            full_name = route + base;

            int file_fd = open(full_name.data(),
                              O_CREAT |     // if file did not exist, create it.
                              O_WRONLY |    // write only.
                              O_TRUNC,      // if file exist, empty it.
                              00700);

            if(file_fd == -1)
            {
               ROS_ERROR(" FILE OPEN FAIL.");
            }else
               ROS_INFO(" FILE OPEN SUCCESS.");


            // write test: 只覆盖了 要写的长度的信息， 弱原文件长度比它长，则后面不修改。
            {
                int write_number;
                char buf[3] = {2, 1, 3};

                write_number = write(file_fd, buf, 3);

                ROS_INFO("WRITE: %d BYTE.", write_number);
            }

            close(file_fd);
        }

        if(0)
        {
            int file_fd = open("/home/cyb7369299/My_Workspace/xyi_v2_ws/src/xyi2_bringup/graphfiles/xyi_graph.data",
                              O_RDONLY      // read only.
                              );

            if(file_fd == -1)
            {
               ROS_ERROR(" FILE OPEN FAIL.");
            }else
               ROS_INFO(" FILE OPEN SUCCESS.");


            {
                int read_number;
                char buf[3] = {0, 0, 0};

                read_number = read(file_fd, buf, 3);

                ROS_INFO("Read: %d BYTE.", read_number);
                ROS_INFO(" DETAILS: %d, %d, %d.",buf[0], buf[1], buf[2]);
            }

            close(file_fd);
        }

        if(0)
        {
            unsigned int int_number = 1000;
            char *char_number;

            char_number = (char *)(&int_number);
            ROS_INFO("CTEST: %d, %d, %d, %d.", char_number[0], char_number[1], char_number[2], char_number[3]);

            unsigned int *show_number;
            show_number = (unsigned int *)char_number;

            ROS_INFO("ITEST: %d.", show_number[0]);
        }

        if(0)
        {
            std::vector<Eigen::Vector3f> test;
            Eigen::Vector3f data;

            data.setZero(); data(0) = 1;
            test.push_back(data);

            data.setZero(); data(1) = 3;
            test.push_back(data);

            data.setZero(); data(1) = 5;
            test.push_back(data);

            int size_pack;
            size_pack = sizeof(float) * 3 * 3;

            char *pack;
            pack = new char[size_pack];

            memcpy(pack, test.data(), size_pack);

            Eigen::Vector3f *data_show;
            data_show = (Eigen::Vector3f *) pack;

            cout << "Vtest: " << endl;
            for(int i=0; i<3; i++)
            {
                cout << data_show[i](0) << ", " << data_show[i](1) << ", " << data_show[i](2) << endl;
            }
        }

        if(0)
        {
            cout << sizeof(Eigen::Vector2f) << endl;    // 4 x 2
        }

        if(0)
        {
            std::vector<int> test;
            test.push_back(4);
            test.push_back(2);
            test.push_back(3);
            test.push_back(1);
            test.push_back(5);
            test.push_back(6);

            test.resize(4);
            for(int i=0; i<4; i++)
                cout << "sss: " << test[i] << endl;
        }

        return(-1);
    }

    // ros::spin();

    return 0;
}

