/* ********************************************************************************
 * author: chenyingbing
 * time: 20170615   10:01   in XIAMEN University
 * illustration:
 *      20170314-20170615: strategy function for mapping.
 *
 * *******************************************************************************/

#include "al_match.h"
#include "search_window_strategy.h"
#include "func_lsr_mapping.h"

#include "../class_tool/hfunc.h"

extern float gpart_temp;

/// *****************************************************************************
/// 1. BruteForce
bool Al_Match::AlMatch_BruteForce(char mode, char lvl_swin, m_lmmapchart *lmmap)
{
    if(sin_chart_init() && lsrdta_deal->isfull())
    {
        /// >-------------------------------------------------
        /// Strategy part:
        static bool r_sta;
        static RM_NORM mat_d;
        static ALM_NODE node_new;
        static float dyaw, c_dyaw, s_dyaw;
        static Eigen::Matrix4d mat_d_m2odom, mat_d_m2lsr;

        static int nid_xy, nid_xyyaw;
        static int i,j,k;

        static vector<vtype_lsr>::iterator it_lsrdta;

        for(i=0; i<SWin_size_x[lvl_swin]; i++)
        {
            for(j=0; j<SWin_size_y[lvl_swin]; j++)
            {
                for(k=0; k<SWin_size_yaw[lvl_swin]; k++)
                {
                    /// get node_new.
                    node_new.node.data()[0] = i;
                    node_new.node.data()[1] = j;
                    node_new.node.data()[2] = k;

                    /// check SWin_varea.
                    nid_xy = node_new.node.data()[1] * SWin_size_x[lvl_swin] + node_new.node.data()[0];
                    nid_xyyaw = node_new.node.data()[2] * SWin_size_xy[lvl_swin] + nid_xy;

                    if(SWin_varea[lvl_swin][nid_xyyaw] > v_default_2)
                    {
                        /// get mat_d_m2lsr.
                        dyaw = (node_new.node.data()[2] - node_s[lvl_swin].node.data()[2]) *
                                SWin_YawReso[lvl_swin];
                        s_dyaw = m_sin(dyaw);
                        c_dyaw = m_cos(dyaw);

                        mat_d.tmatrix.data()[0] = c_dyaw,   mat_d.tmatrix.data()[4] = -s_dyaw;
                        mat_d.tmatrix.data()[1] = s_dyaw;   mat_d.tmatrix.data()[5] = c_dyaw;
                        mat_d.tmatrix.data()[12] = (node_new.node.data()[0] - node_s[lvl_swin].node.data()[0]) *
                                            SWin_XYreso[lvl_swin];
                        mat_d.tmatrix.data()[13] = (node_new.node.data()[1] - node_s[lvl_swin].node.data()[1]) *
                                            SWin_XYreso[lvl_swin];

                        mat_d_m2odom = MOri_matxz * mat_d.tmatrix;
                        mat_d_m2lsr = mat_d_m2odom * MOri_odom;

                        // !!!: lvl is search window lvl, not lsrmatching lvl.
                        it_lsrdta = lsrdta_deal->lsrdta.begin();
                        node_new.values = mtype_lsrmatch(1,
                                                         mode,
                                                         *almatch_srvmap,
                                                         mat_d_m2lsr,
                                                         lmmap,
                                                         lsrdta_deal->isfull(),
                                                         lsrdta_deal->lsrdta_size,
                                                         it_lsrdta);

                        if(node_new.values < node_best[lvl_swin].values)
                        {
                            node_best[lvl_swin].values = node_new.values;
                            node_best[lvl_swin].node = node_new.node;
                            node_best[lvl_swin].gpart = gpart_temp;

                            MCor_matxz = mat_d_m2odom;
                        }

                        /// not be searched yet
                        SWin_checkedq[lvl_swin].push_back(nid_xyyaw);
                        SWin_varea[lvl_swin][nid_xyyaw] = node_new.values;

                    }

                    else{
                        /// have been searched
                        // node_new.values = SWin_varea[lvl_swin][nid_xyyaw];
                    }

                }   // for 3 circles.
            }
        }

        if((mode < mapg_slam_bondary))
        {   // when map exist
            MOri_matxz = MCor_matxz;
            MOri_kf2lsr = MCor_matxz * MOri_odom;

            r_sta = AlMatch_GN_XYClimbing(match_continuous_hmp, 125, lmmap);
        }
        else{
            MOri_matxz = MCor_matxz;
            MOri_kf2lsr = MCor_matxz;

            r_sta = AlMatch_GN_XYClimbing(match_continuous_slam, 125, lmmap);
        }

        /// >-------------------------------------------------
        /// Debug...
        //ROS_INFO("EEEEEE: %d, %d, %d, %d ", SWin_size_x[lvl_swin], SWin_size_y[lvl_swin], SWin_size_yaw[lvl_swin], SWin_size[lvl_swin]);


        return r_sta;
    }

    return false;

}



/// *****************************************************************************
/// 2. GSearch
/*
#define Asearch_NNum    14
static char Search_Neighbor[14][3] = { {1, 0, 1}, {-1, 0, 1}, {0, 0, 1}, {0, 1, 1}, {0, -1, 1},
                                       {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                                       {1, 0, -1}, {-1, 0, -1}, {0, 0, -1}, {0, 1, -1}, {0, -1, -1}
                                     };

void Al_Match::func_gsearch_recursively(char lvl_swin,
                                        char mode,
                                        m_lmmapchart *lmmap,
                                        Eigen::Vector3i &node_now)
{
    static bool isbetter;
    static Eigen::Vector3i dnode_new;

    static float value_subnow = v_default;
    static Eigen::Vector3i dnode_subbest = node_now;

    static int nid_xy, nid_xyyaw;

    static RM_NORM mat_d;
    static float dyaw, c_dyaw, s_dyaw;
    static Eigen::Matrix4d mat_d_m2odom, mat_d_m2lsr;

    static char i;
    static float values;

    static vector<vtype_lsr>::iterator it_lsrdta;

    if(sin_chart_init())
    {
        isbetter = false;

        for(i=0; i<Asearch_NNum; i++)
        {
            dnode_new.data()[0] = node_now.data()[0] + Search_Neighbor[i][0];
            dnode_new.data()[1] = node_now.data()[1] + Search_Neighbor[i][1];
            dnode_new.data()[2] = node_now.data()[2] + Search_Neighbor[i][2];

            // ignore the sample node beyond the SWin range
            if((dnode_new.data()[0] < 0) ||
                (dnode_new.data()[0] >= SWin_size_x[lvl_swin]))
                    continue;

            if((dnode_new.data()[1] < 0) ||
                (dnode_new.data()[1] >= SWin_size_y[lvl_swin]))
                    continue;

            if((dnode_new.data()[2] < 0) ||
                (dnode_new.data()[2] >= SWin_size_yaw[lvl_swin]))
                    continue;

            nid_xy = dnode_new.data()[1] * SWin_size_x[lvl_swin] + dnode_new.data()[0];
            nid_xyyaw = dnode_new.data()[2] * SWin_size_xy[lvl_swin] + nid_xy;

            if(SWin_varea[lvl_swin][nid_xyyaw] > v_default_2)
            {   /// this node is unchecked
                ++node_count;

                /// get mat_d_m2lsr.
                dyaw = (dnode_new.data()[2] - node_s[lvl_swin].node.data()[2]) * SWin_YawReso[lvl_swin];

                s_dyaw = m_sin(dyaw);
                c_dyaw = m_cos(dyaw);
                mat_d.tmatrix.data()[0] = c_dyaw,   mat_d.tmatrix.data()[4] = -s_dyaw;
                mat_d.tmatrix.data()[1] = s_dyaw;   mat_d.tmatrix.data()[5] = c_dyaw;

                mat_d.tmatrix.data()[12] = (dnode_new.data()[0] - node_s[lvl_swin].node.data()[0]) * SWin_XYreso[lvl_swin];
                mat_d.tmatrix.data()[13] = (dnode_new.data()[1] - node_s[lvl_swin].node.data()[1]) * SWin_XYreso[lvl_swin];

                mat_d_m2odom = MOri_matxz * mat_d.tmatrix;
                mat_d_m2lsr = mat_d_m2odom * MOri_odom;

                // !!!: lvl is search window lvl, not lsrmatching lvl.
                it_lsrdta = lsrdta_deal->lsrdta.begin();
                values = mtype_lsrmatch(1,
                                        mode,
                                        *almatch_srvmap,
                                        mat_d_m2lsr,
                                        lmmap,
                                        lsrdta_deal->isfull(),
                                        lsrdta_deal->lsrdta_size,
                                        it_lsrdta);

                SWin_varea[lvl_swin][nid_xyyaw] = values;
                SWin_checkedq[lvl_swin].push_back(nid_xyyaw);

                if(values < node_best[lvl_swin].values)
                {
                    isbetter = true;

                    node_best[lvl_swin].values = values;
                    node_best[lvl_swin].node = dnode_new;
                    node_best[lvl_swin].gpart = gpart_temp;

                    MCor_matxz = mat_d_m2odom;
                }

                if(values <= value_subnow)
                {
                    value_subnow = values;
                    dnode_subbest = dnode_new;
                }

            }//else
                // this node has been checked


        }

        if(isbetter)
        {   /// search more deepest
            iterate_count = 0;
            func_gsearch_recursively(lvl_swin,
                                     mode,
                                     lmmap,
                                     node_best[lvl_swin].node);

        }else{
            /// search sub deepest
            ++iterate_count;
            if(iterate_count < iterate_max)
                func_gsearch_recursively(lvl_swin,
                                         mode,
                                         lmmap,
                                         dnode_subbest);

        }
    }
}

bool Al_Match::AlMatch_GSearch(char mode, m_lmmapchart *lmmap)
{
    if(sin_chart_init() && lsrdta_deal->isfull())
    {
        /// >-------------------------------------------------
        /// Strategy part:

        // Lvl_high Part:
        char lvl_swin = Lvl_HighReso;

        iterate_max = 10;
        iterate_count = 0;

        func_gsearch_recursively(lvl_swin,
                                 mode,
                                 lmmap,
                                 node_best[lvl_swin].node);

        /// >-------------------------------------------------
        /// Debug...
        //ROS_INFO("[AlMatch_GSearch]");

        return true;
    }

    return false;
}
*/

/// *****************************************************************************
/// 3. Gauss-Newton Climbing.
/// Levenberg-Marquardt Method

extern Eigen::Matrix3f Lsrmapping_Hessian;
extern Eigen::Vector3f Lsrmapping_Jacobian, Lsrmapping_dxyz;

extern bool slam_stop;

bool Al_Match::AlMatch_GN_XYClimbing(char mode, int iterates_t, m_lmmapchart *lmmap)
{
    if(sin_chart_init() && lsrdta_deal->isfull())
    {
        /// >-------------------------------------------------
        /// Strategy part:
        #define alpha_max   120
        static Eigen::Matrix3f I, dI;
        static Eigen::Vector3f hlm;
        static float dyaw;

        static float value_o, values;
        static float step_bonus, step_bonus_yaw;
        static float step_alpha, step_alpha_yaw;

        static Eigen::Matrix4d mat_d_odom, mat_d_kf2lsr;
        static vector<vtype_lsr>::iterator it_lsrdta;
        static char lvl = Lvl_HighReso;

        value_o = 1.0f;
        step_alpha = 1.0f,  step_bonus = 0;
        step_alpha_yaw = 1.0f, step_bonus_yaw = 0;

        I.setZero();
        I.data()[0] = 1.0f, I.data()[4] = 1.0f, I.data()[8] = 1.0f;
        dI = I;

        mat_d_odom = MOri_odom;
        mat_d_kf2lsr = MOri_kf2lsr;

        // cout << endl << "[ ";

        while(iterates_t > 0)
        {
            it_lsrdta = lsrdta_deal->lsrdta.begin();
            values = mtype_lsrmatch(lvl,
                                    mode,
                                    *almatch_srvmap,
                                    mat_d_kf2lsr,
                                    lmmap,
                                    lsrdta_deal->isfull(),
                                    lsrdta_deal->lsrdta_size,
                                    it_lsrdta);

            // (J^T * J + alpha * I) * hlm = -J^T * f;
            dI.data()[0] = I.data()[0] * step_alpha;
            dI.data()[4] = I.data()[4] * step_alpha;
            dI.data()[8] = I.data()[8] * step_alpha_yaw;
            hlm = (Lsrmapping_Hessian + dI).inverse() * Lsrmapping_dxyz;

            if( values < value_o)
            {
                step_alpha = step_alpha - 0.95f,    step_bonus = 0;
                step_alpha_yaw = step_alpha_yaw - 0.95f,    step_bonus_yaw = 0;

                if(step_alpha < 0.75f)   step_alpha = 0.75f;
                if(step_alpha_yaw < 0.75f)   step_alpha_yaw = 0.75f;

                value_o = values;
            }

            else{
                --iterates_t;
                step_alpha = step_alpha + 1.0f + step_bonus,   step_bonus += 0.5f;
                step_alpha_yaw = step_alpha_yaw + 1.0f + step_bonus_yaw,    step_bonus_yaw += 0.3f;
            }

            mat_d_kf2lsr.data()[12] += hlm.data()[0];
            mat_d_kf2lsr.data()[13] += hlm.data()[1];
            dyaw = std::atan2(mat_d_kf2lsr.data()[1], mat_d_kf2lsr.data()[0]) + hlm.data()[2];

            if(dyaw > 3.1415926575f) dyaw -= 6.2831853f;
            if(dyaw < -3.1415926575f) dyaw += 6.2831853f;

            mat_d_kf2lsr.data()[0] = m_cos(dyaw);
            mat_d_kf2lsr.data()[1] = m_sin(dyaw);
            mat_d_kf2lsr.data()[4] = -mat_d_kf2lsr.data()[1];
            mat_d_kf2lsr.data()[5] = mat_d_kf2lsr.data()[0];

            if(step_alpha > alpha_max) step_alpha = alpha_max;
            if(step_alpha_yaw > alpha_max) step_alpha_yaw = alpha_max;
            if ((step_alpha == alpha_max) && (step_alpha_yaw == alpha_max))
                break;

            /*
            cout << "[" << step_alpha << ","
                        << step_alpha_yaw << ","
                        << mat_d_kf2lsr.data()[12] << ","
                        << mat_d_kf2lsr.data()[13] << ","
                        << dyaw
                 << "], ";*/

        }

        // cout << "]" << endl;

        MCor_matxz = mat_d_kf2lsr * mat_d_odom.inverse();

        perform_match.giveup_part = gpart_temp;
        perform_match.setdata(values, MCor_matxz, Lsrmapping_Hessian);

        /// >-------------------------------------------------
        /// Debug...

        return true;
    }
    return false;
}


/*
// *****************************************************************************************************
/// *****************************************************************************
/// func brute_force_queue

int Al_Match::func_classify_nodes(unsigned char que_len,
                                  std::vector<ALM_NODE> &nodes_best)
{
    static int class_num;
    static int i,j;

    int *class_node;
    class_node = new int[que_len];

    class_num = 0;

    for(i=0; i<que_len; i++)
        class_node[i] = -1;

    class_node[0] = class_num;

    for(i=0; i<que_len; i++)
    {
        if(class_node[i] == -1)
        {
            ++class_num;
            class_node[i] = class_num;
        }

        for(j=i+1; j<que_len; j++)
        {
            if( (std::abs(nodes_best[j].node.data()[0] - nodes_best[i].node.data()[0]) <= 1) &&
                (std::abs(nodes_best[j].node.data()[1] - nodes_best[i].node.data()[1]) <= 1) &&
                (std::abs(nodes_best[j].node.data()[2] - nodes_best[i].node.data()[2]) <= 1)
              )
            {
                class_node[j] = class_node[i];
            }
        }
    }

    class_num+=1;

    for(i=0; i<class_num; i++)
    {
        for(j=0; j<que_len; j++)
        {
            if(class_node[j] == i)
            {
                nodes_best[i] = nodes_best[j];
                break;
            }
        }
    }

    delete class_node;
    return class_num;
}

void Al_Match::func_nodes_best_reinit(void)
{
    static int cn;

    nodes_qlen = 4;
    nodes_best.clear();
    nodes_best.resize(nodes_qlen);
    for(cn=0; cn<nodes_qlen; cn++)
        nodes_best[cn].values = nodes_value_default;
}

bool Al_Match::func_nodes_best_check(void)
{
    static int cn;

    for(cn=0; cn<nodes_qlen; cn++)
    {
        if(nodes_best[cn].values == nodes_value_default)
            return false;
    }

    return true;
}

void Al_Match::func_brute_force_queue(char lvl,
                                      Struct_Map &srv_map,
                                      Model_Laser &srv_lsrmodel,
                                      sensor_msgs::LaserScanConstPtr &lsr_data)
{
    static RM mat_d;
    static ALM_NODE node_new;
    static float dyaw, c_dyaw, s_dyaw;
    static Eigen::Matrix4d mat_d_m2lsr;

    static vector<ALM_NODE>::iterator it;

    func_nodes_best_reinit();

    if(sin_chart_init())
    {
        static int i,j,k,n;

        for(i=0; i<SWin_size_x[lvl]; i++)
        {
            for(j=0; j<SWin_size_y[lvl]; j++)
            {
                for(k=0; k<SWin_size_yaw[lvl]; k++)
                {
                    /// get node_new.
                    node_new.node.data()[0] = i;
                    node_new.node.data()[1] = j;
                    node_new.node.data()[2] = k;

                    /// get mat_d_m2lsr.
                    dyaw = (node_new.node.data()[2] - node_s[lvl].node.data()[2]) *
                            SWin_YawReso[lvl];
                    s_dyaw = m_sin(dyaw);
                    c_dyaw = m_cos(dyaw);

                    mat_d.tmatrix.data()[0] = c_dyaw,   mat_d.tmatrix.data()[4] = -s_dyaw;
                    mat_d.tmatrix.data()[1] = s_dyaw;   mat_d.tmatrix.data()[5] = c_dyaw;
                    mat_d.tmatrix.data()[12] = (node_new.node.data()[0] - node_s[lvl].node.data()[0]) *
                                        SWin_XYreso[lvl];
                    mat_d.tmatrix.data()[13] = (node_new.node.data()[1] - node_s[lvl].node.data()[1]) *
                                        SWin_XYreso[lvl];

                    mat_d_m2lsr = node_s[lvl].tmat_m2lsr * mat_d.tmatrix;

                    node_new.values = lsr_match(lvl,
                                                srv_map,
                                                srv_lsrmodel,
                                                mat_d_m2lsr,
                                                lsr_data);

                    for(it=nodes_best.begin(), n=0; it!=nodes_best.end(); it++, n++)
                    {
                        if(node_new.values < (*it).values)
                        {
                            node_new.gpart = gpart_temp;
                            node_new.tmat_m2lsr = mat_d_m2lsr;

                            nodes_best.insert(nodes_best.begin()+n, node_new);
                            nodes_best.pop_back();
                            break;
                        }
                    }

                }
            }
        }
    }

}

*/



