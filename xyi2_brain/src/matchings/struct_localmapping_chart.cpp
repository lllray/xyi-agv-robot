/* ********************************************************************************
 * author: chenyingbing
 * time: 20170519   16:08   in XIAMEN University
 * illustration:
 *      local mapping chart.
 *
 * *******************************************************************************/

#include "struct_localmapping_chart.h"

#include <boost/thread.hpp>
#include <boost/ref.hpp>
#include <iostream>
using namespace boost;

#define _lvl    1
#define lnmap_unkown    0.0f

/* ----------------------------------------------------------------------------------------
[0, 0]: 1.000000; [0, 1]: 0.882497; [0, 2]: 0.606531; [0, 3]: 0.324652; [0, 4]: 0.135335;
[1, 0]: 0.882497; [1, 1]: 0.778801; [1, 2]: 0.535261; [1, 3]: 0.286505; [1, 4]: 0.119433;
[2, 0]: 0.606531; [2, 1]: 0.535261; [2, 2]: 0.367879; [2, 3]: 0.196912; [2, 4]: 0.082085;
[3, 0]: 0.324652; [3, 1]: 0.286505; [3, 2]: 0.196912; [3, 3]: 0.105399; [3, 4]: 0.0439369;
[4, 0]: 0.135335; [4, 1]: 0.119433; [4, 2]: 0.082085; [4, 3]: 0.0439369; [4, 4]: 0.0183156;
---------------------------------------------------------------------------------------- */
static int dlength;
static float sqrt_sigma_persize;
static float **value_chart;

void m_lmmapchart::fresh_value_chart(void)
{
    static int j_x, j_y, j;
    static float dis_sigma, p;

    static bool init_value_chart = false;


    static int dlength_c;

    // delete value_chart
    if(init_value_chart)
    {
        for(int i=0; i<(dlength_c+1); i++)
        {
            delete value_chart[i];
        }
        delete value_chart;

        init_value_chart = true;
    }

    // alloc value_chart
    value_chart = new float*[dlength+1];
    for(int i=0; i<(dlength+1); i++)
    {
        value_chart[i] = new float[dlength+1];
    }
    dlength_c = dlength;

    // fill the chart.
    ///cout << endl;
    #define k_steep     1.5f

    for(j_x = 0; j_x <= dlength; j_x++)
    {
        for(j_y = 0; j_y <= dlength; j_y++)
        {
            j = j_x * j_x + j_y * j_y;

            dis_sigma = -0.5f * j * sqrt_sigma_persize;
            p = std::exp(dis_sigma * k_steep);

            value_chart[j_x][j_y] = p;
            ///cout << "[" << j_x << ", " << j_y << "]: " << p << "; ";
        }
        ///cout << endl;
    }
}

void m_lmmapchart::reset(void)
{
    static int dsize;
    dsize = nmap_cord.size();

    static int i;
    for(i=0; i<dsize; i++)
    {
        nmap[nmap_cord[i]] = lnmap_unkown;
        map_pub.data[nmap_cord[i]] = lnmap_unkown;
    }

    nmap_cord.clear();
}


static volatile bool builmap_thread_on = false;

void m_lmmapchart::buildmap(unsigned int nid,
                            Struct_Map *srv_map,
                            mi_lsrdta *mi_lsrin)
{
    if( mi_lsrin->isfull )
    {
        node_id = nid;

        /// alloc the map if haven't
        if(hasalloc == false)
        {
            lreso = mi_lsrin->lreso;
            _lreso = mi_lsrin->_lreso;

            llength = mi_lsrin->llength;
            lsize = mi_lsrin->lsize;

            map_origin = mi_lsrin->map_origin;
            map_xiuz = mi_lsrin->map_xiuz;

            nmap.resize(lsize, lnmap_unkown);
            nmap_cord.clear();

            map_pub.header.frame_id = "/kframe_r";
            map_pub.info.resolution = lreso;
            map_pub.info.width = llength;
            map_pub.info.height = llength;

            map_pub.info.origin.position.x = map_origin(0);
            map_pub.info.origin.position.y = map_origin(1);
            map_pub.info.origin.position.z = 0;

            map_pub.data.resize(lsize, lnmap_unkown);

            hasalloc = true;
        }

        /// fresh value_chart
        dlength = int((srv_map->sigama * 2) * _lreso);  // 2 sigma
        sqrt_sigma_persize = 2.0f/dlength,
        sqrt_sigma_persize = sqrt_sigma_persize * sqrt_sigma_persize;
        fresh_value_chart();

        if(builmap_thread_on == false)
        {
            builmap_thread_on = true;

            boost::thread thread_buildmap_task = boost::thread(boost::bind(&m_lmmapchart::BUILDMAP_TASK, this, mi_lsrin));
        }

    }

    //else
        //nmap_pubcheck(1);
}

void m_lmmapchart::BUILDMAP_TASK(mi_lsrdta *mi_lsrin)
{
    #define k_nmap2pubmap   255

    static int i;
    static Eigen::Vector2i _lixy, _lixy_c, lixy;
    static int j_x, j_y;

    static float fill_p, nmap_p;
    static int lid;

    _lixy_c(0, 0);

    for(i=0; i<mi_lsrin->lsrdta_size; i++)
    {
        _lixy = mi_lsrin->dtas[i];

        for(j_x = -dlength; j_x <= dlength; j_x++)
        {
            for(j_y = -dlength; j_y <= dlength; j_y++)
            {
                lixy.data()[0] = _lixy.data()[0] + j_x;
                lixy.data()[1] = _lixy.data()[1] + j_y;

                if ( ((lixy.data()[0]>=0)&&(lixy.data()[0]<llength)) &&
                ((lixy.data()[1]>=0)&&(lixy.data()[1]<llength))
                )
                {
                    lid = lixy.data()[1] * llength + lixy.data()[0];

                    fill_p = value_chart[std::abs(j_x)][std::abs(j_y)];
                    nmap_p = nmap[lid];

                    /*
                    if(nmap_p == lnmap_unkown)
                    {
                        nmap_cord.push_back(lid);
                    }*/

                    if(fill_p > nmap_p)
                    {
                        nmap[lid] = fill_p;
                        map_pub.data[lid] = int(fill_p * k_nmap2pubmap);
                    }

                }
            }
        }


    }

    // Time cost: 2~8 ms including alloc part. (4592 grids)
    //ROS_INFO("[Localmap_Build]: %d ", debug_t);

    builmap_thread_on = false;
}

// ================================================================================================================
// > NMap Part
void m_lmmapchart::nmap_pubcheck(void)
{
    if(hasalloc)
    {
        map_pub.header.stamp = ros::Time::now();

        static ros::NodeHandle nstru;
        static ros::Publisher datapub = nstru.advertise<nav_msgs::OccupancyGrid>("nmap_mapping",1);
        datapub.publish(map_pub);
    }
}

/// ****************************************************************************************************
/// Sub Threads
/*
static volatile bool thread_buildmap_busy = false;
void m_lmmapchart::new_thread_buildmap(unsigned int nid,
                                       Struct_Map *srv_map,
                                       mi_lsrdta *mi_lsrin)
{


    //if(thread_buildmap_busy)

    boost::thread *thread_task;
    boost::function0<void> trun_task;

    trun_task = boost::bind(&m_lmmapchart::buildmap,this, nid, srv_map, mi_lsrin);
    thread_task = new boost::thread(trun_task);


}*/


