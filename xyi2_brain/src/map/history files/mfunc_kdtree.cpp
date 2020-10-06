#include "struct_map.h"

#include "../class_tool/hfunc.h"

/* ********************************************************************************
 * author: chenyingbing
 * time: 20170508   ??:??   in XIAMEN University
 * illustration:
 *      the map in kdtree struct
 *
 * *******************************************************************************/

/*
void Struct_Map::Kdmap_AddLsr(Model_Laser &srv_lsrmodel,
                              SLAM_DATA &slampack,
                              RM &mat_o2lsr,
                              sensor_msgs::LaserScanConstPtr& lsr_data)
{
    if ((map_isalloc)&&(sin_chart_init()))
    {
        static int ddd;
        static int i, add_i = 1;

        static Eigen::Matrix4d mat_d_m2lsr;
        static float ang, range;

        double time_s;
        boost::timer t_caculate;

        ddd = 0;
        mat_d_m2lsr = slampack.Msta_m2odom * mat_o2lsr.tmatrix;

        for(i=0; i<srv_lsrmodel.lsr_size; i+=add_i)
        {
            // This model ignores max range readings
            if(lsr_data->ranges[i] >= srv_lsrmodel.range_max)
              continue;

            // Check for NaN
            if(lsr_data->ranges[i] != lsr_data->ranges[i])
              continue;

            ang = srv_lsrmodel.angle_range[0] + i*srv_lsrmodel.angle_increment;
            range = lsr_data->ranges[i];

            if ((ang > srv_lsrmodel.ANGLE_RANGE_LIMIT) || (ang < - srv_lsrmodel.ANGLE_RANGE_LIMIT))
                continue;

            static Eigen::Vector2i ixy;
            static float x_d, y_d;
            static float px, py, pz;

            x_d = range * m_cos(ang);
            y_d = range * m_sin(ang);

            px = mat_d_m2lsr.data()[0] * x_d + mat_d_m2lsr.data()[4] * y_d + mat_d_m2lsr.data()[12];
            py = mat_d_m2lsr.data()[1] * x_d + mat_d_m2lsr.data()[5] * y_d + mat_d_m2lsr.data()[13];
            /// pz = mat_d_m2lsr.data()[2] * x_d + mat_d_m2lsr.data()[6] * y_d;

            /// if(std::abs(pz) < 0.05f)
            {
                static char lvl;
                static int x1, y1, lid1;
                static Eigen::Vector2i ixy_c[2];
                static kdtree_node *node_c[2];

                for (lvl=0; lvl<2; lvl++)
                {
                    x1 = lwidth[lvl] + int((px + map_origin[lvl].position.x)/lreso[lvl]);
                    y1 = lheight[lvl] + int((py + map_origin[lvl].position.y)/lreso[lvl]);

                    if ( ((x1>=0)&&(x1<lwidth[lvl])) &&
                         ((y1>=0)&&(y1<lheight[lvl]))
                       )
                    {
                        ixy.data()[0] = x1,
                        ixy.data()[1] = y1;

                        if((ixy == ixy_c[lvl])&&(node_c[lvl] != NULL))
                        {
                            node_c[lvl]->value += 1;
                            ++ddd;  // 1275
                        }else{
                            node_c[lvl] = kd_tree_insert_node(kdtree_map[lvl],
                                                              NULL,
                                                              kdtree_map[lvl]->root,
                                                              ixy);
                            ixy_c[lvl] = ixy;

                            //++ddd;  // 271
                        }

                        /// debug used
                        if(kd_map_show)
                        {
                            lid1 = y1 * lwidth[lvl] + x1;
                            kd_map[lvl].data[lid1] = 100;
                        }
                    }
                }
            }
        }

        // Time cost 1~2 ms.    >> 5~7ms

        time_s = t_caculate.elapsed();
        ROS_INFO("[Kdmap_AddLsr]: %lf, %d | %d, %d , %d, %d", time_s, ddd,
                                                         kdtree_map[0]->depth_count, kdtree_map[0]->leaf_count,
                                                         kdtree_map[1]->depth_count, kdtree_map[1]->leaf_count);


        if(kd_map_show)
            kdmap_sta = true;

    }
}

void Struct_Map::Kdmap_Clear(void)
{
    kdtree_clear(kdtree_map[0]);
    kdtree_clear(kdtree_map[1]);
}

void Struct_Map::KdMap_Pubcheck(char lvl)
{
    /// >-- map pub
    if (kdmap_sta)
    {
        static ros::NodeHandle nstru;
        static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("kdmap_test",1);
        datapub.publish(kd_map[lvl]);
    }
}

*/


