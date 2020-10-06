#include "unit_node.h"

#define lnmap_unkown    0.0f

#define nmap_posi_max   10.0f
#define nmap_obs_posi   0.25f
#define nmap_path_posi  -0.2f

#define nmap_obs_init   2.5f
#define nmap_cond_obstacle 2.19f

#define nmap_posibile_obstacle  0.0f

#define idhistory_unkown -1

void unit_node::alloc_init(Struct_Map *srv_map, m_lsrdta *lsr_in)
{
    if(lsrdta.isfull == false)
    {
        lsrdta.lsrdta_size = 0;
        lsrdta.lsrmax_range = lsr_in->lsrmax_range;
        lsrdta.dtas.clear();

        float d1 = (lsr_in->lsrmax_range + 0.1f);
        int i1 = std::ceil(d1 / srv_map->lreso[_lvl]);
        int i2 = i1 * 2;

        lsrdta.lreso = srv_map->lreso[_lvl];
        lsrdta._lreso = 1.0f / lsrdta.lreso;

        lsrdta.llength = i2;
        lsrdta.lsize = lsrdta.llength * lsrdta.llength;

        lsrdta.map_origin(0) = -(i1 + 0.5f) * lsrdta.lreso;
        lsrdta.map_origin(1) = -(i1 + 0.5f) * lsrdta.lreso;

        lsrdta.map_xiuz(0) = -lsrdta.map_origin(0);
        lsrdta.map_xiuz(1) = -lsrdta.map_origin(1);

        // V.2ed add
        nmap_obs_chart.resize(lsrdta.lsize, false);
        nmap.resize(lsrdta.lsize, lnmap_unkown);
    }
}

bool unit_node::if_available(Eigen::Matrix4d &mat2start, Eigen::Matrix4d &mat2target)
{
    static int x0, y0, x1, y1;
    static int lid, lid0, lid1;
    static int dx, dy;
    static int twice_dx, twice_dy;

    static int x,y;
    static int ad_x, ad_y;
    static int eps;
    static int ad_width;

    static bool r_sta; r_sta = true;

    x0 = std::floor((mat2start.data()[12] + lsrdta.map_xiuz(0)) * lsrdta._lreso);
    y0 = std::floor((mat2start.data()[13] + lsrdta.map_xiuz(1)) * lsrdta._lreso);

    x1 = std::floor((mat2target.data()[12] + lsrdta.map_xiuz(0)) * lsrdta._lreso);
    y1 = std::floor((mat2target.data()[13] + lsrdta.map_xiuz(1)) * lsrdta._lreso);

    lid0 = y0 * lsrdta.llength + x0;
    lid1 = y1 * lsrdta.llength + x1;

    if(nmap[lid0] >= nmap_posibile_obstacle)
        r_sta = false;
    if(nmap[lid1] >= nmap_posibile_obstacle)
        r_sta = false;

    /*
    if( ((x0>=0)&&(x0<lsrdta.llength)) && ((x1>=0)&&(x1<lsrdta.llength)) &&
        ((y0>=0)&&(y0<lsrdta.llength)) && ((y1>=0)&&(y1<lsrdta.llength))
      )
    {
        lid0 = y0 * lsrdta.llength + x0;
        lid1 = y1 * lsrdta.llength + x1;

        lid = lid0;

        dx = std::abs(x1-x0);
        dy = std::abs(y1-y0);

        twice_dx = dx << 1;
        twice_dy = dy << 1;

        ad_x = (((x1 - x0) > 0) << 1) - 1;
        ad_y = (((y1 - y0) > 0) << 1) - 1;

        x = x0, y = y0;
        ad_width = lsrdta.llength * ad_y;

        if(dx > dy)
        {
            eps = twice_dy - dx;
            for (x = x0; x != x1; x += ad_x) {
                /// draw
                if(nmap[lid] >= nmap_posibile_obstacle)
                    r_sta = false;

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
                /// draw
                if(nmap[lid] >= nmap_posibile_obstacle)
                    r_sta = false;

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

    }else
        return false;
    */

    return r_sta;
}


// when init: add laserdata to origin node;
void unit_node::add_lsrdata(Struct_Map *srv_map,
                            Eigen::Matrix4d &mat_kf2lsr, m_lsrdta *lsr_in)
{
    alloc_init(srv_map, lsr_in);

    static vtype_lsr pxy, xy_d;
    static Eigen::Vector2i _lixy, _lixy_c;

    static int lid;

    {
        for(int i=0; i<lsr_in->lsrdta_size; i++)
        {
            xy_d.lsr_fxy(0) = lsr_in->lsrdta[i].lsr_fxy(0);
            xy_d.lsr_fxy(1) = lsr_in->lsrdta[i].lsr_fxy(1);

            pxy.lsr_fxy(0) = mat_kf2lsr.data()[0] * xy_d.lsr_fxy(0) + mat_kf2lsr.data()[4] * xy_d.lsr_fxy(1) + mat_kf2lsr.data()[12];
            pxy.lsr_fxy(1) = mat_kf2lsr.data()[1] * xy_d.lsr_fxy(0) + mat_kf2lsr.data()[5] * xy_d.lsr_fxy(1) + mat_kf2lsr.data()[13];

            _lixy(0) = std::floor((pxy.lsr_fxy(0) + lsrdta.map_xiuz(0)) * lsrdta._lreso);
            _lixy(1) = std::floor((pxy.lsr_fxy(1) + lsrdta.map_xiuz(1)) * lsrdta._lreso);

            if( (_lixy != _lixy_c) &&
                ( ((_lixy(0)>=0)&&(_lixy(0)<lsrdta.llength)) &&
                  ((_lixy(1)>=0)&&(_lixy(1)<lsrdta.llength)) )
              )
            {
                lsrdta.dtas.push_back(_lixy);
                ++lsrdta.lsrdta_size;

                lid = _lixy(1) * lsrdta.llength + _lixy(0);

                nmap[lid] = nmap_obs_init;
                nmap_obs_chart[lid] = true;
            }

            _lixy_c = _lixy;
        }
    }

    lsrdta.isfull = true;

}

// when normal freshing: add laserdata to node;
void unit_node::add_lsrdata_v2(bool enable_fresh,
                               Struct_Map *srv_map,
                               Eigen::Matrix4d &mat_kf2lsr, m_lsrdta *lsr_in)
{
    alloc_init(srv_map, lsr_in);

    static vtype_lsr pxy, xy_d;
    static Eigen::Vector2i _lixy, _lixy_c;

    static int x0, y0, nx1, ny1, x1, y1;
    static int lid, lid0, lid1;
    static int dx, dy;
    static int twice_dx, twice_dy;

    static int x,y;
    static int ad_x, ad_y;
    static int eps;
    static int ad_width;

    static int i;
    static std::vector<int> lid1_cord;
    static std::vector<Eigen::Vector2i> lid1_xy_cord;

    static std::vector<vtype_lsr> lid1_lsrxy_cord;
    static node_lsrmes lsrmes;

    static bool ismidbreak;

    lid1_cord.clear();
    lid1_xy_cord.clear();
    lid1_lsrxy_cord.clear();

    {
        x0 = std::floor((mat_kf2lsr.data()[12] + lsrdta.map_xiuz(0)) * lsrdta._lreso);
        y0 = std::floor((mat_kf2lsr.data()[13] + lsrdta.map_xiuz(1)) * lsrdta._lreso);

        for(i=0; i<lsr_in->lsrdta_size; i++)
        {
            xy_d.lsr_fxy(0) = lsr_in->lsrdta[i].lsr_fxy(0);
            xy_d.lsr_fxy(1) = lsr_in->lsrdta[i].lsr_fxy(1);

            pxy.lsr_fxy(0) = mat_kf2lsr.data()[0] * xy_d.lsr_fxy(0) + mat_kf2lsr.data()[4] * xy_d.lsr_fxy(1) + mat_kf2lsr.data()[12];
            pxy.lsr_fxy(1) = mat_kf2lsr.data()[1] * xy_d.lsr_fxy(0) + mat_kf2lsr.data()[5] * xy_d.lsr_fxy(1) + mat_kf2lsr.data()[13];

            _lixy(0) = std::floor((pxy.lsr_fxy(0) + lsrdta.map_xiuz(0)) * lsrdta._lreso);
            _lixy(1) = std::floor((pxy.lsr_fxy(1) + lsrdta.map_xiuz(1)) * lsrdta._lreso);

            if( (_lixy != _lixy_c) &&
                ( ((_lixy(0)>=0)&&(_lixy(0)<lsrdta.llength)) &&
                  ((_lixy(1)>=0)&&(_lixy(1)<lsrdta.llength)) )
              )
            {
                x1 = _lixy.data()[0],
                y1 = _lixy.data()[1];

                nx1 = std::max(x0, x1 - 3);
                ny1 = std::max(y0, y1 - 3);

                lid0 = y0 * lsrdta.llength + x0;
                lid1 = y1 * lsrdta.llength + x1;

                lid = lid0;

                dx = std::abs(x1-x0);
                dy = std::abs(y1-y0);

                twice_dx = dx << 1;
                twice_dy = dy << 1;

                ad_x = (((x1 - x0) > 0) << 1) - 1;
                ad_y = (((y1 - y0) > 0) << 1) - 1;

                x = x0, y = y0;
                ad_width = lsrdta.llength * ad_y;

                ismidbreak = false;

                if(dx > dy)
                {
                    eps = twice_dy - dx;
                    for (x = x0; x != x1; x += ad_x) {
                        if((x > nx1)&&(nmap_obs_chart[lid] == true))
                        {
                            ismidbreak = true;
                            break;
                        }

                        /// draw
                        if(nmap[lid] > -nmap_posi_max)
                            nmap[lid] += nmap_path_posi;

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
                        if((y > ny1)&&(nmap_obs_chart[lid] == true))
                        {
                            ismidbreak = true;
                            break;
                        }

                        /// draw
                        if(nmap[lid] > -nmap_posi_max)
                            nmap[lid] += nmap_path_posi;

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

                /// draw
                if(ismidbreak == false)
                    if(nmap[lid1] < nmap_posi_max)
                        nmap[lid1] += nmap_obs_posi;

                if(enable_fresh)
                {
                    if( nmap[lid1] > nmap_cond_obstacle)
                    {
                        lid1_cord.push_back(lid1);
                        lid1_xy_cord.push_back(Eigen::Vector2i(x1, y1));
                        lid1_lsrxy_cord.push_back(xy_d);
                    }
                }
            }

            _lixy_c = _lixy;
        }

        if(enable_fresh)
        {
            for(i=0; i<lid1_cord.size(); i++)
            {
                if( (nmap[lid1_cord[i]] > nmap_cond_obstacle) && (nmap_obs_chart[lid1_cord[i]] == false) )
                {

                    lsrdta.dtas.push_back(lid1_xy_cord[i]);
                    ++lsrdta.lsrdta_size;

                    nmap_obs_chart[lid1_cord[i]] = true;
                }
            }
        }
    }

    lsrdta.isfull = true;
}


// when init new file_node: add laserdata to new frame_node.
bool unit_node::add_lsrdata_v3(unit_node &base_node,
                               Struct_Map *srv_map,
                               Eigen::Matrix4d &mat_bnode2lsr, m_lsrdta *lsr_in)
{
    static RM_NORM Imat;

    if(base_node.lsrdta.isfull == false)
        return false;

    alloc_init(srv_map, lsr_in);

    static vtype_lsr pxy, xy_d;
    static Eigen::Vector2i _lixy, _lixy_c;

    static int lid, lid0, lid1;
    static int x0, y0, x1, y1;

    static int i;
    static node_lsrmes lsrmes;

    static Eigen::Vector2i local_lixy;

    {
        // x0 = std::floor((mat_bnode2lsr.data()[12] + lsrdta.map_xiuz(0)) * lsrdta._lreso);
        // y0 = std::floor((mat_bnode2lsr.data()[13] + lsrdta.map_xiuz(1)) * lsrdta._lreso);

        for(i=0; i<lsr_in->lsrdta_size; i++)
        {
            xy_d.lsr_fxy(0) = lsr_in->lsrdta[i].lsr_fxy(0);
            xy_d.lsr_fxy(1) = lsr_in->lsrdta[i].lsr_fxy(1);

            pxy.lsr_fxy(0) = mat_bnode2lsr.data()[0] * xy_d.lsr_fxy(0) + mat_bnode2lsr.data()[4] * xy_d.lsr_fxy(1) + mat_bnode2lsr.data()[12];
            pxy.lsr_fxy(1) = mat_bnode2lsr.data()[1] * xy_d.lsr_fxy(0) + mat_bnode2lsr.data()[5] * xy_d.lsr_fxy(1) + mat_bnode2lsr.data()[13];

            _lixy(0) = std::floor((pxy.lsr_fxy(0) + lsrdta.map_xiuz(0)) * lsrdta._lreso);
            _lixy(1) = std::floor((pxy.lsr_fxy(1) + lsrdta.map_xiuz(1)) * lsrdta._lreso);

            if( (_lixy != _lixy_c) &&
                ( ((_lixy(0)>=0)&&(_lixy(0)<lsrdta.llength)) &&
                  ((_lixy(1)>=0)&&(_lixy(1)<lsrdta.llength)) )
              )
            {
                x1 = _lixy.data()[0],
                y1 = _lixy.data()[1];

                // lid0 = y0 * lsrdta.llength + x0;
                lid1 = y1 * lsrdta.llength + x1;

                if(base_node.nmap[lid1] > nmap_cond_obstacle)
                {
                    local_lixy(0) = std::floor((xy_d.lsr_fxy(0) + lsrdta.map_xiuz(0)) * lsrdta._lreso); // !!!
                    local_lixy(1) = std::floor((xy_d.lsr_fxy(1) + lsrdta.map_xiuz(1)) * lsrdta._lreso);

                    lid = local_lixy(1) * lsrdta.llength + local_lixy(0);

                    lsrdta.dtas.push_back(local_lixy);
                    ++lsrdta.lsrdta_size;

                    nmap[lid] = nmap_obs_init;   // !!!
                    nmap_obs_chart[lid] = true;
                }
            }

            _lixy_c = _lixy;
        }

    }

    lsrdta.isfull = true;

    return true;
}















