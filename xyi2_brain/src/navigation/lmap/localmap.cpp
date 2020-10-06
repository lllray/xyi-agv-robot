/* ********************************************************************************
 * author: chenyingbing
 * time: 20171102   14:07   in XIAMEN University
 * illustration:
 *      mange the local dynamic map.
 *
 * *******************************************************************************/

#include "localmap.h"

#define obsmap_default              0
#define const_k_mdismap_show        100.0f

////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////

void local_obsmap::get_map_p3ilocation(Eigen::Matrix4d &matpose_map, Eigen::Vector3i &v3i_out)
{
    v3i_out(0) = std::floor((matpose_map.data()[12] + map_xiuz.position.x) * inverse_map_reso);
    v3i_out(1) = std::floor((matpose_map.data()[13] + map_xiuz.position.y) * inverse_map_reso);
    v3i_out(2) = v3i_out(1) * map_width + v3i_out(0);
}

////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////

void local_obsmap::obsmap_init(float _map_reso,
                               int _map_width, int _map_height, int _map_size,
                               geometry_msgs::Pose _map_xiuz, float _mdismap_range)
{
    if(map_size_c != _map_size)
    {
        map_size_c = _map_size;

        map_reso = _map_reso,   inverse_map_reso = 1.0f / map_reso;
        map_width = _map_width;
        map_height = _map_height;
        map_size = _map_size;
        map_xiuz.position.x = _map_xiuz.position.x,
            map_xiuz.position.y = _map_xiuz.position.y;

        mdismap_range = _mdismap_range;

        mdismap_show.info.resolution = map_reso;
        mdismap_show.info.width = map_width;
        mdismap_show.info.height = map_height;
        mdismap_show.info.origin.position.x = - _map_xiuz.position.x,
         mdismap_show.info.origin.position.y = - _map_xiuz.position.y;

        mdis_default = Disnode_Unkown;

        mdismap.resize(map_size, mdis_default);
        mdismap_show.data.resize(map_size, obsmap_default);

        has_init = true;
    }
}

void local_obsmap::localmap_fresh(Eigen::Matrix4d &matpose_m2lsr, beam_lsrdta &_blsrdta)
{
    #define localmap_pi_2   1.5706f
    #define localmap_pi     3.1415926f
    #define degree2radian   0.01745f

    static int i;

    static float beam_get;
    static float x_d, y_d, yaw_d;
    static float px, py;
    static float sin_yaw_d, cos_yaw_d;

    static Eigen::Vector3i v3i_deal;

    // mdismap.
    static int j, k, l, d, d_c;
    static int mid_d;
    static bool sta;
    static float dis_pix;

    static int dd;
    static std::vector<int> addlocates;
    static std::vector<float> adddiss;

    static std::vector<int> mdismap_oldnodes;

    double time_s;
    boost::timer t_caculate;

    if(has_init && sin_chart_init())
    {
        d = std::ceil(mdismap_range / map_reso);

        if(d != d_c)
        {
            d_c = d;
            dd = (d*2+1); dd = dd * dd;
            addlocates.clear();
            adddiss.clear();
            for(k=-d; k<=d; k++)
                for(l=-d; l<=d; l++)
                {
                    addlocates.push_back( (l*map_width + k) );
                    adddiss.push_back( std::sqrt(k*k + l*l) * map_reso);
                }
        }

        for(j=0; j<mdismap_oldnodes.size(); j++)
        {
            mdismap[mdismap_oldnodes[j]] = mdis_default;
            mdismap_show.data[mdismap_oldnodes[j]] = obsmap_default;
        }
        mdismap_oldnodes.clear();

        for(i=0; i<_blsrdta.beam_size; i++)
        {
            yaw_d = (i - _blsrdta.beam_size_2) * degree2radian;

            if( (yaw_d > localmap_pi_2) || (yaw_d < -localmap_pi_2) ) continue;

            sin_yaw_d = m_sin(yaw_d);
            cos_yaw_d = m_cos(yaw_d);

            beam_get = _blsrdta.beam_range[i];

            x_d = beam_get * cos_yaw_d;
            y_d = beam_get * sin_yaw_d;

            px = matpose_m2lsr.data()[0] * x_d + matpose_m2lsr.data()[4] * y_d + matpose_m2lsr.data()[12];
            py = matpose_m2lsr.data()[1] * x_d + matpose_m2lsr.data()[5] * y_d + matpose_m2lsr.data()[13];

            v3i_deal(0) = std::floor((px + map_xiuz.position.x) * inverse_map_reso);
            v3i_deal(1) = std::floor((py + map_xiuz.position.y) * inverse_map_reso);
            v3i_deal(2) = v3i_deal(1) * map_width + v3i_deal(0);

            if( (beam_get > _blsrdta.default_disline) && (beam_get < localmap_max_range) )
                if ( ((v3i_deal(0)>=0)&&(v3i_deal(0)<map_width)) &&
                     ((v3i_deal(1)>=0)&&(v3i_deal(1)<map_height))
                   )
                {
                    for(k=0; k<dd; k++)
                    {
                        mid_d = v3i_deal(2) + addlocates[k];
                        sta = ((mid_d >=0 )&&(mid_d < map_size));

                        if(sta)
                        {
                            dis_pix = adddiss[k];

                            if(dis_pix < mdismap[mid_d])
                            {
                                mdismap[mid_d] = dis_pix;
                                mdismap_show.data[mid_d] = dis_pix * const_k_mdismap_show;

                                mdismap_oldnodes.push_back(mid_d);
                            }
                        }
                        else
                            continue;
                    }

                }

        }
    }

    // time_s = t_caculate.elapsed() * 1000;
    // ROS_INFO("[LocalMap Construct]: %.1lf ms", time_s);

}



