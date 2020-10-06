/* ********************************************************************************
 * author: chenyingbing
 * time: 20170512   14:55   in XIAMEN University
 * illustration:
 *      ???:        lsr mapping using old message type:  sensor_msg::laserScan.
 *      20170512:   lsr mapping using new message type:  lsr_filter.h
 *
 * *******************************************************************************/

#include "func_lsr_mapping.h"
#include "../class_tool/hfunc.h"

/// type: all
float gpart_temp;

/// type_continuous
Eigen::Matrix3f Lsrmapping_Hessian;
Eigen::Vector3f Lsrmapping_Jacobian, Lsrmapping_dxyz;


/// ****************************************************************************************************
/// mtype_lsrmatch
//  ****************************************************************************************************

static bool nmap_value_type;
inline static float nmap_value(Struct_Map &srv_map,
                               m_lmmapchart *srv_lmmapchart,
                               char lvlmap_mapexist,
                               int lid)
{
    return nmap_value_type ? srv_map.lnmap[lvlmap_mapexist][lid] : srv_lmmapchart->nmap[lid];
}

inline static void vector3f_setlimit(Eigen::Vector3f &data_in, float *limit)
{
    static char i;

    for(i=0; i<3; i++)
    {
        if(data_in(i) > limit[i])        data_in(i) = limit[i];
        else if(data_in(i) < -limit[i])  data_in(i) = -limit[i];
    }
}

float mtype_lsrmatch(char lvlmap_mapexist,
                     char mode,
                     Struct_Map &srv_map,
                     Eigen::Matrix4d &dmat_kf2lsr,
                     m_lmmapchart *srv_lmmapchart,
                     bool isfull,
                     int lsr_size,
                     vector<vtype_lsr>::iterator &it_lsrdta)
{
    #define isstrong_threhold   0.1f
    #define ignore_threhold     0.25f

    if(sin_chart_init() && isfull)
    {
        static float m_reso, m_reso_2, _m_reso;
        static int m_width, m_height;
        static float deal_xz_x, deal_xz_y;

        static int i;
        static float ave_p;

        static float px, py, pz;
        static float x_d, y_d;

        static bool maping_mode;  /// true: map exsits; false: slam mode.
        static char maping_method;

        static int unknown_part;
        static float p;

        maping_mode = (mode <= mapg_slam_bondary);
        maping_method = (mode % 100);

        unknown_part = 0;
        ave_p = 0;

        Lsrmapping_Jacobian.setZero();
        Lsrmapping_dxyz.setZero();
        Lsrmapping_Hessian.setZero();

        // mode: map exist.
        if(maping_mode == true)
        {
            nmap_value_type = true;

            m_reso = srv_map.lreso[lvlmap_mapexist];
            m_reso_2 = m_reso * 0.5f;

            m_width = srv_map.lwidth[lvlmap_mapexist];
            m_height = srv_map.lheight[lvlmap_mapexist];

            deal_xz_x = srv_map.map_xiuz[lvlmap_mapexist].position.x;
            deal_xz_y = srv_map.map_xiuz[lvlmap_mapexist].position.y;
            _m_reso = 1.0f / m_reso;

        }

        // mode: slam.
        else{
            nmap_value_type = false;

            m_reso = srv_lmmapchart->lreso;
            m_reso_2 = srv_lmmapchart->lreso * 0.5f;

            m_width = srv_lmmapchart->llength;
            m_height = srv_lmmapchart->llength;

            deal_xz_x = srv_lmmapchart->map_xiuz(0);
            deal_xz_y = srv_lmmapchart->map_xiuz(1);
            _m_reso = 1.0f / srv_lmmapchart->lreso;

        }

        static Eigen::Vector2i lixy;
        static int lid;
        static float p_x, p_y;
        static int t_x[2], t_y[2];

        if(maping_method == tpye_normal)
        {
            for(i=0; i<lsr_size; i+=1, it_lsrdta+=1)
            {
                x_d = it_lsrdta->lsr_fxy(0);
                y_d = it_lsrdta->lsr_fxy(1);

                px = dmat_kf2lsr.data()[0] * x_d + dmat_kf2lsr.data()[4] * y_d + dmat_kf2lsr.data()[12];
                py = dmat_kf2lsr.data()[1] * x_d + dmat_kf2lsr.data()[5] * y_d + dmat_kf2lsr.data()[13];
                /// pz = dmat_kf2lsr.data()[2] * x_d + dmat_kf2lsr.data()[6] * y_d;

                // ********************************************************
                lixy.data()[0] = std::floor((px + deal_xz_x) * _m_reso);
                lixy.data()[1] = std::floor((py + deal_xz_y) * _m_reso);

                if ( ((lixy.data()[0]>=0)&&(lixy.data()[0]<m_width)) &&
                     ((lixy.data()[1]>=0)&&(lixy.data()[1]<m_height))
                   )
                {
                    lid = lixy.data()[1] * m_width + lixy.data()[0];

                    p = nmap_value(srv_map, srv_lmmapchart,
                                   lvlmap_mapexist, lid);
                    // p = srv_lmmapchart->nmap[lid];

                    /* statistic in type_continuous
                    if(it_lsrdta->isstrong == false)
                        ++unknown_part;
                    else if(p < isstrong_threhold)
                    {
                        ++unknown_part;
                        it_lsrdta->isstrong = false;
                    }*/

                    ave_p += p;
                }
            }
        }

        else if(maping_method == type_continuous)
        {
            /// get 4 points value.
            static float tf_x[2], tf_y[2];
            static int lid[4];
            static float p[4];
            static float pp, _pp;

            /// difference value.
            /// Reduce caculation amount.
            static float a0, a1, b0, b1;
            static float a0p0_plus_a1p1, a0p3_plus_a1p2;
            static float b0p0, b0p1, b1p3, b1p2;

            /// Get Hessian.
            static float da0 = -1, da1 = 1, db0 = -1, db1 = 1;
            static float dyaw_a0 , dyaw_a1, dyaw_b0, dyaw_b1;
            static float deal[3];

            for(i=0; i<lsr_size; i+=1, it_lsrdta+=1)
            {
                x_d = it_lsrdta->lsr_fxy(0);
                y_d = it_lsrdta->lsr_fxy(1);

                px = dmat_kf2lsr.data()[0] * x_d + dmat_kf2lsr.data()[4] * y_d + dmat_kf2lsr.data()[12];
                py = dmat_kf2lsr.data()[1] * x_d + dmat_kf2lsr.data()[5] * y_d + dmat_kf2lsr.data()[13];
                /// pz = dmat_kf2lsr.data()[2] * x_d + dmat_kf2lsr.data()[6] * y_d;

                // ********************************************************
                //    2       1    t_x[1]
                //                          add_x = 1 or -1;
                //    3       0    t_x[0]
                //  t_y[1]  t_y[0]
                //       add_y = 1 or -1;

                p_x = (px + deal_xz_x);
                p_y = (py + deal_xz_y);

                p_x -= m_reso_2;    // !!!
                p_y -= m_reso_2;

                t_x[0] = std::floor(p_x * _m_reso);
                t_y[0] = std::floor(p_y * _m_reso);

                t_x[1] = t_x[0] + 1;
                t_y[1] = t_y[0] + 1;

                if ( ((t_x[0]>=0)&&(t_x[1]<m_width)) &&
                     ((t_y[0]>=0)&&(t_y[1]<m_height))
                   )
                {
                    /// get 4 points value.

                    tf_x[0] = t_x[0] * m_reso,
                     tf_y[0] = t_y[0] * m_reso;
                    tf_x[1] = tf_x[0] + m_reso,
                     tf_y[1] = tf_y[0] + m_reso;

                    lid[0] = t_y[0] * m_width + t_x[0];
                    lid[1] = lid[0] + 1;
                    lid[2] = lid[1] + m_width;
                    lid[3] = lid[0] + m_width;

                    p[0] = nmap_value(srv_map, srv_lmmapchart, lvlmap_mapexist, lid[0]);
                    p[1] = nmap_value(srv_map, srv_lmmapchart, lvlmap_mapexist, lid[1]);
                    p[2] = nmap_value(srv_map, srv_lmmapchart, lvlmap_mapexist, lid[2]);
                    p[3] = nmap_value(srv_map, srv_lmmapchart, lvlmap_mapexist, lid[3]);

                    /// difference value.
                    /// Reduce caculation amount.

                    a0 = (tf_x[1] - p_x) * _m_reso;
                    a1 = (p_x - tf_x[0]) * _m_reso;
                    b0 = (tf_y[1] - p_y) * _m_reso;
                    b1 = (p_y - tf_y[0]) * _m_reso;

                    a0p0_plus_a1p1 = (a0 * p[0] + a1 * p[1]);
                    a0p3_plus_a1p2 = (a0 * p[3] + a1 * p[2]);
                    b0p0 = b0 * p[0];
                    b0p1 = b0 * p[1];
                    b1p3 = b1 * p[3];
                    b1p2 = b1 * p[2];

                    pp = ( b0 * a0p0_plus_a1p1 +
                           b1 * a0p3_plus_a1p2
                         );
                    _pp = 1.0f - pp;

                    if(it_lsrdta->isstrong == false)
                        ++unknown_part;
                    else if(pp < isstrong_threhold)
                    {
                        ++unknown_part;
                        it_lsrdta->isstrong = false;
                    }

                    if(pp < ignore_threhold)  // ignore terrible beam.
                        continue;

                    /// Get Hessian.

                    dyaw_a0 = -(x_d * dmat_kf2lsr.data()[4] - y_d * dmat_kf2lsr.data()[0]);
                    dyaw_a1 = -dyaw_a0;
                    dyaw_b0 = -(x_d * dmat_kf2lsr.data()[0] + y_d * dmat_kf2lsr.data()[4]);
                    dyaw_b1 = -dyaw_b0;

                    deal[0] = ( (da0 * b0p0 + da1 * b0p1) +
                                (da0 * b1p3 + da1 * b1p2)
                              );
                    deal[1] = ( db0 * a0p0_plus_a1p1 +
                                db1 * a0p3_plus_a1p2
                              );
                    deal[2] = ( dyaw_b0 * a0p0_plus_a1p1 + (dyaw_a0 * b0p0 + dyaw_a1 * b0p1) +
                                dyaw_b1 * a0p3_plus_a1p2 + (dyaw_a0 * b1p3 + dyaw_a1 * b1p2)
                              );

                    Lsrmapping_Jacobian.data()[0] += deal[0];
                    Lsrmapping_Jacobian.data()[1] += deal[1];
                    Lsrmapping_Jacobian.data()[2] += deal[2];

                    Lsrmapping_dxyz.data()[0] += deal[0] * _pp;
                    Lsrmapping_dxyz.data()[1] += deal[1] * _pp;
                    Lsrmapping_dxyz.data()[2] += deal[2] * _pp;

                    Lsrmapping_Hessian.data()[0] += deal[0]*deal[0],   //            [3]                        //            [6]
                    Lsrmapping_Hessian.data()[1] += deal[1]*deal[0],   Lsrmapping_Hessian.data()[4] += deal[1]*deal[1],    //            [7]
                    Lsrmapping_Hessian.data()[2] += deal[2]*deal[0],   Lsrmapping_Hessian.data()[5] += deal[2]*deal[1],    Lsrmapping_Hessian.data()[8] += deal[2]*deal[2];

                    /*
                    Lsrmapping_Hessian.data()[0] += deal[0]*deal[0],   //            [3]                                   //            [6]
                                                                       Lsrmapping_Hessian.data()[4] += deal[1]*deal[1],    //            [7]
                                                                                                                           Lsrmapping_Hessian.data()[8] += deal[2]*deal[2];
                    */

                    ave_p += pp;
                }

            }
        }

        static float rlimit[3] = {0.1f, 0.1f, 0.15f};

        if(lsr_size > 0)
        {
            if(maping_method == type_continuous)
            {   /// map exists.

                Lsrmapping_Hessian.data()[3] = Lsrmapping_Hessian.data()[1],
                 Lsrmapping_Hessian.data()[6] = Lsrmapping_Hessian.data()[2],
                  Lsrmapping_Hessian.data()[7] = Lsrmapping_Hessian.data()[5];

                /// !!! very important!
                Lsrmapping_Jacobian /= lsr_size;
                Lsrmapping_Hessian /= lsr_size;

                Lsrmapping_dxyz.data()[0] /= (lsr_size * 1.0f);   // get
                Lsrmapping_dxyz.data()[1] /= (lsr_size * 1.0f);   // get
                Lsrmapping_dxyz.data()[2] /= (lsr_size * 1.5f);   // get
                vector3f_setlimit(Lsrmapping_dxyz, rlimit);
            }

            gpart_temp = float(unknown_part) / lsr_size;
            ave_p /= lsr_size;

            return (1.0f - ave_p);
        }

    }

    return 0;
}


