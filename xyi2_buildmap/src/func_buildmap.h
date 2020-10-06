#ifndef __func_buildmap_H
#define __func_buildmap_H

    // Sys Part **********************************************/
    #include <ros/ros.h>
    #include <sstream>
    #include <iostream>
    #include <string>
    using namespace std;

    // C_Plus Standard Library *******************************/
    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    // TF Part ***********************************************/
    #include "tf/transform_broadcaster.h"
    #include "tf/transform_listener.h"

    // Message Part ******************************************/
    #include "nav_msgs/OccupancyGrid.h"
    #include "geometry_msgs/PoseArray.h"
    #include "geometry_msgs/PolygonStamped.h"

    // Math Part *********************************************/
    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    // Opencv Part *******************************************/
    #include <cv.h>
    #include <cv_bridge/cv_bridge.h>
    #include <image_transport/image_transport.h>
    //#include <opencv2/imgproc/imgproc.hpp>
    //#include <opencv2/highgui/highgui.hpp>
    #include <sensor_msgs/image_encodings.h>
    #include <image_transport/image_transport.h>

    #include "graph_io/graph_io.h"

    enum Mnode_type{Mnode_Unkown=-1, Mnode_Path=0, Mnode_Obstacle=100};

    struct MAP_PARAM_BLOCK
    {
    public:

        MAP_PARAM_BLOCK()
        {

        }

        float map_lengthofsize;
        float map_resolution;

    private:

    };

    typedef class MAP_MANAGE
    {
    public:
        MAP_MANAGE()
        {
            map_param_init = false;

            bmap_sta = false;
            nmap_sta = false;

            fbmap_sta = false;

            graph_poses.poses.clear();

        }

        ~MAP_MANAGE()
        {

        }

        void func_map_param_init(MAP_PARAM_BLOCK &pram_set);
        bool build_map(bool mode);

        void nmap_pubcheck(void);
        void bmap_pubcheck(void);
        void graph_poses_pubcheck(void);
        void graph_edges_pubcheck(void);

    private:

        bool map_param_init;
        MAP_PARAM_BLOCK map_pram;

        int map_size, map_width, map_height;
        float map_reso, _map_reso;
        Eigen::Vector2f map_xiuz;

        bool nmap_sta;
        std::vector<float> nmap;

        bool bmap_sta;
        nav_msgs::OccupancyGrid bmap;

        bool fbmap_sta;
        nav_msgs::OccupancyGrid fbmap;

        bool edge_map_sta;
        geometry_msgs::PoseArray graph_poses;
        nav_msgs::OccupancyGrid graph_edge_map;

        void AllMap_Init(void);
        void Graph_Update_Nmap(void);
        void Nmap_Update_Bmap(void);
        void Bmap_Update_FBmap(void);

    }MAP_MANAGE;

    extern MAP_MANAGE map_manage;

#endif










