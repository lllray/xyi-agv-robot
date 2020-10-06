/* ********************************************************************************
 * author: chenyingbing
 * time: 20170512   14:55   in XIAMEN University
 * illustration:
 *      The mapping functions when the map has existed.
 *
 * *******************************************************************************/

#include "struct_map.h"

#include "../class_tool/hfunc.h"

/* ********************************************************************************
 * author: chenyingbing
 * time: 20170313-20170330   21:25, 16:27   in XIAMEN University
 * illustration:
 *      the map has two levels resolution.
 *
 * *******************************************************************************/

bool Struct_Map::SlamInit_ExistMap(const nav_msgs::OccupancyGridConstPtr &map_s)
{
    if ((map_isalloc)&&(sin_chart_init()))
    {
        int i,j;
        float d1, d2;
        /// Reset the message.
        lreso[1] = map_s->info.resolution;
        lwidth[1] = map_s->info.width;
        lheight[1] = map_s->info.height;

        pow_lreso[0] = lreso[0] * lreso[0];

        lmap[1].info.resolution = lreso[1];
        lmap[1].info.width = lwidth[1];
        lmap[1].info.height = lheight[1];
        lsize[1] = lwidth[1] * lheight[1];

        lreso[0] = lreso[1] * (L0_Expand / L1_Expand);
        lwidth[0] = lwidth[1] / (L0_Expand / L1_Expand);
        lheight[0] = lheight[1] / (L0_Expand / L1_Expand);

        pow_lreso[1] = lreso[1] * lreso[1];

        map_size[0] = map_s->info.width * map_s->info.resolution;
        map_size[1] = map_s->info.height * map_s->info.resolution;

        lmap[0].info.resolution = lreso[0];
        lmap[0].info.width = lwidth[0];
        lmap[0].info.height = lheight[0];
        lsize[0] = lwidth[0] * lheight[0];

        for (i=0; i<2; i++)
        {
            lmap[i].data.resize(lsize[i], Mnode_Unkown);

            d1 = (lwidth[i] * 0.5f);
            d2 = (lheight[i] * 0.5f);

            lmap[i].info.origin.position.x = -(d1 + 0.5f) * lreso[i];
            lmap[i].info.origin.position.y = -(d2 + 0.5f) * lreso[i];
            lmap[i].info.origin.position.z = 0;

            map_origin[i] = lmap[i].info.origin;

            map_xiuz[i].position.x = -map_origin[i].position.x;
            map_xiuz[i].position.y = -map_origin[i].position.y;

        }

        float fx, fy;
        int did;
        int cx = 0, cy = 0, id = 0;

        for(i=0; i<lsize[1]; i++, cx++, id++)
        {
            /// cx = j%width_in;                // x id in map_in
            ///  cy = j/width_in;                // y id in map_in
            if(cx>=lwidth[1])
            {
                cx -= lwidth[1];
                cy += 1;
            }

            /// lvl0
            fx = cx*_L0_Expand, fy = cy*_L0_Expand;
            did = std::floor(fy)*lwidth[0] + std::floor(fx);
            if(map_s->data[id] > lmap[0].data[did])
            {
                lmap[0].data[did] = map_s->data[id];
            }

            lmap[1].data[id] = map_s->data[id];
        }

        lmap_sta = true;

        return true;
    }
    return false;
}

// ================================================================================================================
// > LMap Part

void Struct_Map::LMap_Pubcheck(char lvl)
{
    /// >-- map pub
    if (lmap_sta)
    {
        static ros::NodeHandle nstru;
        static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("lmap_test",1);
        datapub.publish(lmap[lvl]);
    }
}

/* RULES: if (dis > 2 sigama) pro = zero.
 * PROBLEM: None > fix the problem of : p(L) !> p(H)
 * COST: 100ms
 *      - Yingbing, 2017-03-21, In XIAMEN University, China.
*/
void Struct_Map::LMap_UpdateNMap(Model_Laser &srv_lsrmodel)
{
    // Ros::Time 0.0s

    double time_s;
    boost::timer t_caculate;

    if (lmap_sta)
    {
        /// >check the Level 0 and fill the Level 1
        /// Mnode_Unkown=-1, Mnode_Path=0, Mnode_Obs=100
        #define MDIS_DEFAULT    100

        static char llvl = 0;

        static bool sta;
        static int i, j, k, l;
        static int xd, yd, mid, mid_d;

        static float dis_pix;
        static float filldata;

        static int d, d2;

        for(llvl=0; llvl<2; llvl++)
        {
            d = 3 * std::ceil(sigama / lreso[llvl]);
            d2 =  std::ceil(mdismap_range / lreso[llvl]);

            for(i=0; i<lwidth[llvl]; i++)
            {
                for(j=0; j<lheight[llvl]; j++)
                {
                    mid = j*lwidth[llvl] + i;

                    if(lmap[llvl].data[mid] == Mnode_Obs)
                    {
                        // lnmap
                        lnmap[llvl][mid] = 1.0f;

                        for(k=-d; k<=d; k++)
                        {
                            for(l=-d; l<=d; l++)
                            {
                                xd = i + k, yd = j + l;
                                sta = true;
                                sta &= ((xd>=0)&&(yd>=0));
                                sta &= ((xd<lwidth[llvl])&&(yd<lheight[llvl]));

                                if(sta)
                                {
                                    mid_d = yd*lwidth[llvl] + xd;

                                    dis_pix = k*k + l*l;
                                    dis_pix *= powlreso_nigama[llvl];
                                    filldata = srv_lsrmodel.Lsr_Model_Gauss(std::exp(-(dis_pix)));

                                    if(filldata > lnmap[llvl][mid_d])
                                        lnmap[llvl][mid_d] = filldata;
                                }
                                else
                                    continue;
                            }
                        }

                        // mdismap
                        mdismap[llvl][mid] = 0.0f;

                        for(k=-d2; k<=d2; k++)
                        {
                            for(l=-d2; l<=d2; l++)
                            {
                                xd = i + k, yd = j + l;
                                sta = true;
                                sta &= ((xd>=0)&&(yd>=0));
                                sta &= ((xd<lwidth[llvl])&&(yd<lheight[llvl]));

                                if(sta)
                                {
                                    mid_d = yd*lwidth[llvl] + xd;

                                    dis_pix = std::sqrt(k*k + l*l);
                                    dis_pix *= lreso[llvl];
                                    filldata = dis_pix;

                                    if(filldata < mdismap[llvl][mid_d])
                                        mdismap[llvl][mid_d] = filldata;
                                }
                                else
                                    continue;
                            }
                        }

                    }
                }
            }
        }

    }

    nmap_sta = true;
    mdismap_sta = true;

    /// Note: if lnmap has two axis, time_s will cost nearly 300ms!!!
    time_s = t_caculate.elapsed();
    ROS_INFO("[LMap_UpdateNMap]: %lf s", time_s); // totally 100ms
}

// ================================================================================================================
// > NMap Part
void Struct_Map::NMap_Pubcheck(char lvl)
{
    if(nmap_sta)
    {
        unsigned int i;
        static bool int_once = true;
        static nav_msgs::OccupancyGrid map_pub;

        if(int_once)
        {
            map_pub.header.stamp = ros::Time::now();
            map_pub.header.frame_id = "/map_r";
            map_pub.info.resolution = lreso[lvl];
            map_pub.info.width = lwidth[lvl];
            map_pub.info.height = lheight[lvl];
            map_pub.info.origin = map_origin[lvl];

            map_pub.data.resize(lsize[lvl]);

            int deal = 0;
            for(i=0; i<lsize[lvl]; i++)
            {
                deal = lnmap[lvl][i] * 250;
                map_pub.data[i] = deal;
            }
        }

        static ros::NodeHandle nstru;
        static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("nmap_mapping",1);
        datapub.publish(map_pub);
    }
}

// ================================================================================================================
// > MdisMap Part
void Struct_Map::Mdismap_Pubcheck(char lvl)
{
    if(nmap_sta)
    {
        unsigned int i;
        static bool int_once = true;
        static nav_msgs::OccupancyGrid map_pub;

        if(int_once)
        {
            map_pub.header.stamp = ros::Time::now();
            map_pub.header.frame_id = "/map_r";
            map_pub.info.resolution = lreso[lvl];
            map_pub.info.width = lwidth[lvl];
            map_pub.info.height = lheight[lvl];
            map_pub.info.origin = map_origin[lvl];

            map_pub.data.resize(lsize[lvl]);

            int deal = 0;
            for(i=0; i<lsize[lvl]; i++)
            {
                deal = mdismap[lvl][i] * 100;

                if(deal > 255) deal = 255;

                map_pub.data[i] = deal;
            }
        }

        static ros::NodeHandle nstru;
        static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("mdismap",1);
        datapub.publish(map_pub);
    }
}

