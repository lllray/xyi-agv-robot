/* ********************************************************************************
 * author: chenyingbing
 * time: 20170613   20:03   in XIAMEN University
 * illustration:
 *  the basic funcion of graph: astar search.
 *
 * *******************************************************************************/
#include "struct_graph.h"

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

/// ****************************************************************************************************************************
//  ************************** color line... beautiful!! ***********************************************************************
/// ****************************************************************************************************************************
// these functions is used in [unit_table] case.

int list_graph::get_table_locateid(bool &is_inrange, Eigen::Matrix4d &posi_map, Eigen::Vector3i &ixy_r)
{
    static float d;
    static float px, py, pyaw;

    static float q0, q1, q2;

    is_inrange = false;

    if(hasinit)
    {
        px = posi_map.data()[12];
        py = posi_map.data()[13];
        pyaw = atan2(posi_map.data()[1], posi_map.data()[0]);	// A1/A0

        px += map_xiuz(0);
        py += map_xiuz(1);
        pyaw += PI;

        // d      = 0, 1, 2, 3, 4, 5, 6, 7, 8 ...
        // result = 0, 1, 1, 2, 2, 3, 3, 4, 4 ...

        d = std::floor(px / param_interval_x);
        ixy_r(0) = d;

        d = std::floor(py / param_interval_y);
        ixy_r(1) = d;

        d = std::floor(pyaw / param_interval_yaw);
        ixy_r(2) = d;

        px /= param_interval_x;
        py /= param_interval_y;
        pyaw /= param_interval_yaw;

        q0 = std::floor(px);
        q1 = std::floor(py);
        q2 = std::floor(pyaw);

        if( (px>(q0+table_inrange_x[0]) && px<(q0+table_inrange_x[1])) &&
            (py>(q1+table_inrange_y[0]) && py<(q1+table_inrange_y[1])) &&
            (pyaw>(q2+table_inrange_yaw[0]) && pyaw<(q2+table_inrange_yaw[1]))
          )
            is_inrange = true;

        //ROS_INFO("HER: px, py = [%f, %f]", px, py);

        //return (ixy_r(1) * graph_table.width + ixy_r(0));
        return (ixy_r(2) * graph_table.size_xy + ixy_r(1) * graph_table.length + ixy_r(0));

    }

    return -1;
}

/// ****************************************************************************************************************************
//  ************************** color line... beautiful!! ***********************************************************************
/// ****************************************************************************************************************************
// these functions is used in [unit_kdtree] case.


/// ****************************************************************************************************************************
//  ************************** color line... beautiful!! ***********************************************************************
/// ****************************************************************************************************************************

void list_graph::graph_init(Struct_Map *srv_map,
                            m_lsrdta *lsr_filter,
                            float sx_interval, float sy_interval, float syaw_interval)
{
    #define inrange_dis_x   0.1f           // m
    #define inrange_dis_y   0.1f           // m
    #define inrange_angle_yaw   0.08726f    // 5 degree.

    if( (hasinit == false) && srv_map->Map_alloc_sta() && lsr_filter->isfull())
    {
        bool flag_locateid;

        /// alloc data. ***************************************************************
        param_interval_x = sx_interval;
        param_interval_y = sy_interval;
        param_interval_yaw = syaw_interval;

        param_interval_x_2 = param_interval_x * 0.5f;
        param_interval_y_2 = param_interval_y * 0.5f;
        param_interval_yaw_2 = param_interval_yaw * 0.5f;

        map_origin(0) = srv_map->map_origin[_lvl].position.x;
        map_origin(1) = srv_map->map_origin[_lvl].position.y;
        map_xiuz(0) = -map_origin(0);
        map_xiuz(1) = -map_origin(1);

        vertexNum = 0;
        edgeNum = 0;
        graph_nodes.clear();

        hasinit = true;

        /// manage_struct: unit_table. *************************
        table_inrange_x[0] = inrange_dis_x / param_interval_x,
         table_inrange_x[1] = 1.0f - table_inrange_x[0];

        table_inrange_y[0] = inrange_dis_y / param_interval_y,
         table_inrange_y[1] = 1.0f - table_inrange_y[0];

        table_inrange_yaw[0] = inrange_angle_yaw / param_interval_yaw,
         table_inrange_yaw[1] = 1.0f - table_inrange_yaw[0];

        Eigen::Vector3d d_xyyaw(srv_map->lwidth[_lvl] * srv_map->lreso[_lvl]  + srv_map->map_origin[_lvl].position.x,
                                srv_map->lheight[_lvl] * srv_map->lreso[_lvl] + srv_map->map_origin[_lvl].position.y,
                                PI
                                );
        Eigen::Matrix4d upleft_corner;
        XyyawGetMatrix4d(d_xyyaw, upleft_corner);

        Eigen::Vector3i upleft_id;
        get_table_locateid(flag_locateid, upleft_corner, upleft_id);

        graph_table.length  = upleft_id(0) + 1;
        graph_table.width = upleft_id(1) + 1;
        graph_table.height = upleft_id(2) + 1;

        graph_table.size_xy = graph_table.length * graph_table.width;
        graph_table.size = graph_table.size_xy * graph_table.height;
        graph_table.grids.resize(graph_table.size);

        /// manage_struct: unit_kdtree. ************************



        /// add fisrt node. ***********************************************************
        // >fill unit_node
        unit_node fill_node;

        RM_NORM Imat;
        fill_node.mat_source2lsr = lsr_filter->slam_kf2lsr;

        fill_node.node_id = vertexNum++;
        fill_node.locate_id = get_table_locateid(flag_locateid, fill_node.mat_source2lsr, fill_node.locate_xy);

        fill_node.lsrdta.isfull = false;
        fill_node.add_lsrdata(srv_map, Imat.tmatrix, lsr_filter);

        graph_nodes.push_back(fill_node);

        akey_node = 0;
        akey_node_old = 0;

        // >fill unit_table
        table_mes fill_table;

        fill_table.isfilled = true;
        fill_table.node_id = fill_node.node_id;

        graph_table.grids[fill_node.locate_id] = fill_table;

        /// daw table map. ***********************************************************
        #define table_map_color     240

        int i, j, lid;
        int add_i, add_j;
        table_map.header.frame_id = srv_map->lmap[_lvl].header.frame_id;
        table_map.info = srv_map->lmap[_lvl].info;
        table_map.data.resize(srv_map->lsize[_lvl], Mnode_Unkown);

        add_i = std::ceil(sx_interval/srv_map->lreso[_lvl]);
        for(i=0; i<srv_map->lwidth[_lvl]; i+= add_i)
            for(j=0; j<srv_map->lheight[_lvl]; j+=3)
            {
                lid = j * srv_map->lwidth[_lvl] + i;
                table_map.data[lid] = table_map_color;
            }

        i = srv_map->lwidth[_lvl] - 1;
        for(j=0; j<srv_map->lheight[_lvl]; j+=3)
        {
            lid = j * srv_map->lwidth[_lvl] + i;
            table_map.data[lid] = table_map_color;
        }

        add_j = std::ceil(sy_interval/srv_map->lreso[_lvl]);
        for(j=0; j<srv_map->lheight[_lvl]; j+= add_j)
            for(i=0; i<srv_map->lwidth[_lvl]; i+=3)
            {
                lid = j * srv_map->lwidth[_lvl] + i;
                table_map.data[lid] = table_map_color;
            }

        j = srv_map->lheight[_lvl] - 1;
        for(i=0; i<srv_map->lwidth[_lvl]; i+=3)
        {
            lid = j * srv_map->lwidth[_lvl] + i;
            table_map.data[lid] = table_map_color;
        }

        table_map_init = true;

        ROS_INFO(" GRAPH INIT SUCCESS." );

    }
}

void list_graph::slaminteract_map_show(void)
{
    static bool publish_once = true;
    static ros::NodeHandle nstru;
    static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("slam_interactmap",1);

    if(table_map_init && publish_once)
    {
        datapub.publish(table_map);
        publish_once = false;
    }
}






