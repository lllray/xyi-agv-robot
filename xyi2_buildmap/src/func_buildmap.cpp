/* ********************************************************************************
 * author: chenyingbing
 * time: 20170628 15:10  in XIAMEN University
 * illustration:
 *      the function to build the static map according to the graph data.
 *
 * *******************************************************************************/

#include "func_buildmap.h"

MAP_MANAGE map_manage;

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

extern FILE_HEAD file_head;
extern std::vector<FILE_NODE>  file_nodes;         // NODES PACK
extern std::vector<FILE_EDGE>  file_edges;         // EDGES PACK
extern std::vector<float>      file_grids;         // GRIDS PACK

extern FILE_TAIL file_tail;

static float m_rand(float b, float ed)
{
    static float value;

    std::srand((unsigned int)std::time(0));

    value = std::rand();
    value /= RAND_MAX;      /// 0 ~ 1.0f

    value *= (ed - b);      /// 0 ~ (ed-b);
    value += b;             /// b ~ ed

    return value;
}

static Eigen::Matrix4f mat_d2f(Eigen::Matrix4d &mat_in)
{
    static int i;
    static Eigen::Matrix4f r_mat;
    for(i=0; i<16; i++){
        r_mat.data()[i] = mat_in.data()[i];
    }

    return r_mat;
}

static void XyyawGetMatrix4f(Eigen::Vector3f &In_xyyaw, Eigen::Matrix4f &Out_MatrixDta)
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

static void Matrix4fGetGeoPose(Eigen::Matrix4f &MatrixDta, geometry_msgs::Pose &posedta)
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

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

#define Nnode_Unkown    0.0f
float Nnode_Path_Add, Nnode_Obs_Add;

void MAP_MANAGE::func_map_param_init(MAP_PARAM_BLOCK &pram_set)
{
    std::memcpy(&map_pram, &pram_set, sizeof(MAP_PARAM_BLOCK));

    map_param_init = true;
}

/// mode == true, just draw the obstacle.
/// mode == false, will draw the obstacle and path.
bool MAP_MANAGE::build_map(bool mode)
{
    if(map_param_init)
    {
        if(mode)
        {
            Nnode_Path_Add = -0.0f;
            Nnode_Obs_Add = 0.75f;
        }else{
            Nnode_Path_Add = -0.1f;
            Nnode_Obs_Add = 1.0f;
        }

        AllMap_Init();

        ROS_INFO("PROCESS OF BUILDING MAP: Bmap Init.");

        Graph_Update_Nmap();

        ROS_INFO("PROCESS OF BUILDING MAP: Nmap Ready.");

        Nmap_Update_Bmap();

        ROS_INFO("PROCESS OF BUILDING MAP: Bmap Ready.");

        Bmap_Update_FBmap();


        return true;
    }
    else
        ROS_ERROR("build map: MAP PARAMETER HAVE NOT BEEN SET.");


    return false;
}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

void MAP_MANAGE::AllMap_Init(void)
{
    float d1, d2;

    if(map_param_init)
    {
        nmap_sta = false;
        bmap_sta = false;
        edge_map_sta = false;
        fbmap_sta = false;

        // bmap
        bmap.header.frame_id = "/map_r";

        map_reso = map_pram.map_resolution;
        _map_reso = 1.0f / map_reso;
        map_width = round(map_pram.map_lengthofsize / map_pram.map_resolution);
        map_height = map_width;

        bmap.info.resolution = map_reso;
        bmap.info.width = map_width;
        bmap.info.height = map_height;
        map_size = map_width * map_height;

        bmap.info.origin.orientation.w = 1.0f,
         bmap.info.origin.orientation.x = 0.0,
          bmap.info.origin.orientation.y = 0.0,
           bmap.info.origin.orientation.z = 0.0;

        d1 = (map_width * 0.5f);
        d2 = (map_height * 0.5f);

        bmap.info.origin.position.x = -(d1 + 0.5f) * map_reso;
        bmap.info.origin.position.y = -(d2 + 0.5f) * map_reso;
        bmap.info.origin.position.z = 0;

        map_xiuz(0) = -bmap.info.origin.position.x;
        map_xiuz(1) = -bmap.info.origin.position.y;

        bmap.data.clear();
        bmap.data.resize(map_size, Mnode_Unkown);

        fbmap.header.frame_id = bmap.header.frame_id;
        fbmap.info = bmap.info;

        fbmap.data.resize(map_size, Mnode_Unkown);

        // nmap
        nmap.clear();
        nmap.resize(map_size, Nnode_Unkown);

        // edge_map
        graph_edge_map.info = bmap.info;

        graph_edge_map.header.frame_id = "/map_r";

        graph_edge_map.data.clear();
        graph_edge_map.data.resize(map_size, Mnode_Unkown);
    }
    else
        ROS_ERROR("AllMap_Init: MAP PARAMETER HAVE NOT BEEN SET.");
}

void MAP_MANAGE::Graph_Update_Nmap(void)
{
    /* *********************************************************************
    struct FILE_NODE
        unsigned int node_id;               // 4 bytes
        Eigen::Vector3f X;                  // 12 bytes

        unsigned int num_edges;             // 4 bytes
        Eigen::Vector2i edges_id_ft;        // 8 bytes (from (0) to (1))

        unsigned int num_lsrdtas;           // 4 bytes
        Eigen::Vector2i lsrdtas_id_ft;      // 8 bytes (from (0) to (1))
    ********************************************************************* */

    // 00000000000000000000000000000000000000000000000000000000000000000000
    int i, l;
    Eigen::Matrix4f mat_source2kf;
    geometry_msgs::Pose pose_source2kf;

    int grids_x, grids_y;
    float grids_fx, grids_fy;
    float px, py;
    int x1, y1, lid1;
    int lid;

    // 00000000000000000000000000000000000000000000000000000000000000000000

    #define maximum_distance    30.0f
    #define edge_add_max        160
    #define edge_add_constchar  50

    int j;

    int dx, dy;
    int twice_dx, twice_dy;

    int x,y;
    int ad_x, ad_y;
    int eps;
    int ad_width;

    int x0, y0, lid0;

    Eigen::Matrix4f mat_source2kf_d1, mat_source2kf_d2;

    // 00000000000000000000000000000000000000000000000000000000000000000000

    if(map_param_init)
    {
        // for traveling through the node frame.
        for(i = 0; i<file_head.num_nodes; i++)
        {
            XyyawGetMatrix4f(file_nodes[i].X, mat_source2kf);
            Matrix4fGetGeoPose(mat_source2kf, pose_source2kf);

            graph_poses.poses.push_back(pose_source2kf);

            /* nmap fresh.
            unsigned int   nmap_llength;        // 4 bytes
            unsigned int   nmap_size;           // 4 bytes
            float nmap_reso;                    // 4 bytes
            Eigen::Vector2f nmap_xiuz;          // 8 bytes
            Eigen::Vector2i nmap_gid_ft;        // 8 bytes (from (0) to (1))        */

            for(l = file_nodes[i].nmap_gid_ft(0), grids_x = 0, grids_y = 0; l <= file_nodes[i].nmap_gid_ft(1); l++, grids_x++)
            {
                if(grids_x >= file_nodes[i].nmap_llength)
                {
                    grids_x -= file_nodes[i].nmap_llength;
                    grids_y += 1;
                }

                grids_fx = -file_nodes[i].nmap_xiuz(0) + grids_x * file_nodes[i].nmap_reso;
                grids_fy = -file_nodes[i].nmap_xiuz(1) + grids_y * file_nodes[i].nmap_reso;

                px = mat_source2kf.data()[0] * grids_fx + mat_source2kf.data()[4] * grids_fy + mat_source2kf.data()[12];
                py = mat_source2kf.data()[1] * grids_fx + mat_source2kf.data()[5] * grids_fy + mat_source2kf.data()[13];

                x1 = std::floor((px + map_xiuz(0)) * _map_reso);
                y1 = std::floor((py + map_xiuz(1)) * _map_reso);

                if ( ((x1>=0)&&(x1<map_width)) &&
                     ((y1>=0)&&(y1<map_height))
                   )
                {
                    lid = y1 * map_width + x1;

                    nmap[lid] += file_grids[l];
                }
            }

            // edge fresh.
            for(j = file_nodes[i].edges_id_ft(0); j <= file_nodes[i].edges_id_ft(1); j++)
            {
                XyyawGetMatrix4f(file_nodes[file_edges[j].nodes_id_ft(0)].X, mat_source2kf_d1);
                XyyawGetMatrix4f(file_nodes[file_edges[j].nodes_id_ft(1)].X, mat_source2kf_d2);

                x0 = std::floor((mat_source2kf_d1.data()[12] + map_xiuz(0)) * _map_reso);
                y0 = std::floor((mat_source2kf_d1.data()[13] + map_xiuz(1)) * _map_reso);

                x1 = std::floor((mat_source2kf_d2.data()[12] + map_xiuz(0)) * _map_reso);
                y1 = std::floor((mat_source2kf_d2.data()[13] + map_xiuz(1)) * _map_reso);

                if ( ((x1>=0)&&(x1<map_width)) &&
                     ((y1>=0)&&(y1<map_height))
                   )
                {
                    lid0 = y0 * map_width + x0;
                    lid1 = y1 * map_width + x1;

                    lid = lid0;

                    dx = std::abs(x1-x0);
                    dy = std::abs(y1-y0);

                    twice_dx = dx << 1;
                    twice_dy = dy << 1;

                    ad_x = (((x1 - x0) > 0) << 1) - 1;
                    ad_y = (((y1 - y0) > 0) << 1) - 1;

                    x = x0, y = y0;
                    ad_width = map_width * ad_y;

                    if(dx > dy)
                    {
                        eps = twice_dy - dx;
                        for (x = x0; x != x1; x += ad_x) {
                            if(graph_edge_map.data[lid] == Mnode_Unkown) graph_edge_map.data[lid] = Mnode_Path;
                            else if(graph_edge_map.data[lid] < edge_add_max)
                                graph_edge_map.data[lid] += edge_add_constchar;

                            if(eps >= 0)
                            {   /// move to next line
                                eps -= twice_dx;
                                y += ad_y;

                                lid += ad_width;
                            }

                            eps += twice_dy;

                            /// move to next pixel
                            lid += ad_x;
                        }
                    }

                    else{
                        eps = twice_dx - dy;
                        for (y = y0; y != y1; y += ad_y) {
                            if(graph_edge_map.data[lid] == Mnode_Unkown) graph_edge_map.data[lid] = Mnode_Path;
                            else if(graph_edge_map.data[lid] < edge_add_max)
                                graph_edge_map.data[lid] += edge_add_constchar;

                            if(eps >= 0)
                            {   /// move to next line
                                eps -= twice_dy;
                                x += ad_x;

                                lid += ad_x;
                            }

                            eps += twice_dx;

                            /// move to next pixel
                            lid += ad_width;
                        }
                    }

                    if(graph_edge_map.data[lid1] == Mnode_Unkown) graph_edge_map.data[lid1] = Mnode_Path;
                    else if(graph_edge_map.data[lid1] < edge_add_max)
                        graph_edge_map.data[lid1] += edge_add_constchar;
                }
            }

        }

        nmap_sta = true;
        edge_map_sta = true;

    }else
        ROS_ERROR("Graph_Update_Nmap: MAP PARAMETER HAVE NOT BEEN SET.");

}

const int mat_cond_path = 10;
const int mat_cond_obs = 90;

void MAP_MANAGE::Nmap_Update_Bmap(void)
{
    int i;
    float d_e;
    int deal;

    if(map_param_init)
    {
        for(i=0; i<map_size; i++)
        {
            d_e = std::exp(nmap[i]);

            deal = int ((d_e / (1 + d_e))*100.0f);

            if(deal <= mat_cond_path)
                bmap.data[i] = Mnode_Path;
            else if(deal >= mat_cond_obs)
            {
                bmap.data[i] = Mnode_Obstacle;
            }else
                bmap.data[i] = Mnode_Unkown;


        }

        bmap_sta = true;
    }
    else
        ROS_ERROR("Nmap_Update_Bmap: MAP PARAMETER HAVE NOT BEEN SET.");
}

void MAP_MANAGE::Bmap_Update_FBmap(void)
{
    static int i,j, lid;

    cv::Mat matData, matData2, matData3;
    IplImage *matImage, *matImage2, *matImage3;

    uchar *p, *p2, *p3;

    {
        // create space
        matData.create(map_width, map_height, CV_8UC1);
        matData2.create(map_width, map_height, CV_8UC1);
        matData3.create(map_width, map_height, CV_8UC1);

        // copy data to map_s_data->Map_MatDta
        for(j = 0; j < map_height; ++j)
        {
            p = matData.ptr<uchar>(j);
            p2 = matData2.ptr<uchar>(j);
            p3 = matData2.ptr<uchar>(j);

            for (i = 0; i < map_width; ++i )
            {
                lid = (j*map_width + i);

                p[i] = bmap.data[lid];
                p2[i] = bmap.data[lid];
                p3[i] = bmap.data[lid];

            }
        }

        // generate IplImage
        matImage = new IplImage(matData);
        matImage2 = new IplImage(matData2);
        matImage3 = new IplImage(matData3);
    }

    /*
    /// image deal...
    if(0)
    {
        int values[25] = {0,0,1,0,0, 0,0,1,0,0, 1,1,1,1,1, 0,0,1,0,0, 0,0,1,0,0};

        cvDilate(matImage2, matImage2,
                 cvCreateStructuringElementEx(3, 3, // cols, rows
                                              1, 1, // anchor_col, anchor_row
                                              CV_SHAPE_RECT,
                                              NULL),
                 1
                 );

        for(j = 0; j < map_height; ++j)
        {
            p = matData.ptr<uchar>(j);
            p2 = matData2.ptr<uchar>(j);

            for (i = 0; i < map_width; ++i )
            {
                lid = (j*map_width + i);

                if(p[i] < p2[i])
                    p[i] = Mnode_Path;
            }
        }

        // matImage3
        cvErode(matImage, matImage3,
                 cvCreateStructuringElementEx(5, 5, // cols, rows
                                              2, 2, // anchor_col, anchor_row
                                              CV_SHAPE_RECT,
                                              values),
                 1
                 );

        cvDilate(matImage3, matImage3,
                 cvCreateStructuringElementEx(5, 5, // cols, rows
                                              2, 2, // anchor_col, anchor_row
                                              CV_SHAPE_RECT,
                                              values),
                 1
                 );

        for(j = 0; j < map_height; ++j)
        {
            p = matData3.ptr<uchar>(j);

            for (i = 0; i < map_width; ++i )
            {
                if(p[i] > 20)
                    p[i] = Mnode_Unkown;
                else
                    p[i] = Mnode_Path;
            }
        }

        // matImage
        cvDilate(matImage, matImage,
                 cvCreateStructuringElementEx(5, 5, // cols, rows
                                              2, 2, // anchor_col, anchor_row
                                              CV_SHAPE_RECT,
                                              values),
                 1
                 );

        cvErode(matImage, matImage,
                 cvCreateStructuringElementEx(5, 5, // cols, rows
                                              2, 2, // anchor_col, anchor_row
                                              CV_SHAPE_RECT,
                                              values),
                 1
                 );

        for(j = 0; j < map_height; ++j)
        {
            p = matData.ptr<uchar>(j);

            for (i = 0; i < map_width; ++i )
            {
                if(p[i] >= mat_cond_obs)
                    p[i] = Mnode_Obstacle;

                if( (p[i] >= mat_cond_unkown[0]) && (p[i] <= mat_cond_unkown[1]))
                    p[i] = Mnode_Unkown;

                if(p[i] <= mat_cond_path)
                    p[i] = Mnode_Path;
            }
        }

        // matImage3 > matImage
        for(j = 0; j < map_height; ++j)
        {
            p = matData.ptr<uchar>(j);
            p2 = matData3.ptr<uchar>(j);

            for (i = 0; i < map_width; ++i )
            {
                if( (p2[i] == Mnode_Path) && (p[i] != Mnode_Obstacle)
                  )
                {
                    p[i] = Mnode_Path;
                }
            }
        }
    }
    */

    // reset obstacle and copy to pubmap show
    for(j = 0; j < map_height; ++j)
    {
        p = matData.ptr<uchar>(j);

        for (i = 0; i < map_width; ++i )
        {
            lid = (j*map_width + i);

            fbmap.data[lid] = p[i];

            /*
            if((bmap.data[lid] == Mnode_Unkown))
                fbmap.data[lid] = Mnode_Unkown;
            else
                fbmap.data[lid] = p[i];*/
        }
    }

    matData.release();  matData2.release();  matData3.release();
    delete matImage, matImage2, matImage3;

    fbmap_sta = true;

}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

void MAP_MANAGE::nmap_pubcheck(void)
{
    static bool init_once = true;
    static ros::NodeHandle nstru;
    static ros::Publisher datapub;

    static unsigned int i;
    static nav_msgs::OccupancyGrid map_pub;

    if(nmap_sta)
    {
        map_pub.header.stamp = ros::Time::now();


        int deal = 0;
        if(init_once)
        {
            map_pub.header.frame_id = bmap.header.frame_id;
            map_pub.info = bmap.info;

            map_pub.data.resize(map_size, Mnode_Unkown);

            for(i=0; i<map_size; i++)
            {
                deal = int(std::exp(nmap[i] / (std::exp(nmap[i] + 1))) * 100.0f);
                map_pub.data[i] = deal;

            }

            init_once = false;
        }

        datapub = nstru.advertise<nav_msgs::OccupancyGrid>("grpah_opt_nmap",1);
        datapub.publish(map_pub);
    }
}

void MAP_MANAGE::bmap_pubcheck(void)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub;

    #define show_map    fbmap

    if(fbmap_sta)
    {
        show_map.header.stamp = ros::Time::now();

        datapub = nstru.advertise<nav_msgs::OccupancyGrid>("grpah_opt_bmap",1);
        datapub.publish(show_map);
    }
}

void MAP_MANAGE::graph_poses_pubcheck(void)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub;

    graph_poses.header.frame_id = "/map_r";

    if(nmap_sta)
    {
        graph_poses.header.stamp = ros::Time::now();

        datapub = nstru.advertise<geometry_msgs::PoseArray>("grpah_poses",1);
        datapub.publish(graph_poses);
    }
}

void MAP_MANAGE::graph_edges_pubcheck(void)
{
    static ros::NodeHandle nstru;
    static ros::Publisher datapub;

    if(edge_map_sta)
    {
        ROS_INFO(" GRPAH_EDGES: More brighten, More num of edges.");
        ROS_INFO(" Ref: Transparent > 1 edge, Purple > 2 edges, Yellow > 3 edges and so on.");

        graph_edge_map.header.stamp = ros::Time::now();

        datapub = nstru.advertise<nav_msgs::OccupancyGrid>("grpah_edges",1);
        datapub.publish(graph_edge_map);
    }
}


