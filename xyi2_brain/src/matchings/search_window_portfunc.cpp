/* ********************************************************************************
 * author: chenyingbing
 * time: 20170615   10:01   in XIAMEN University
 * illustration:
 *      20170314-20170615: port function for mapping.
 *
 * *******************************************************************************/

#include "al_match.h"

extern bool slam_stop;
extern float gpart_temp;

/// ***************************************************************************************************
//* I am a line, please don't erase me!!!! ************************************************************
/// ***************************************************************************************************

bool Al_Match::AlMatch_Mapping(bool slam_enable)
{
    static bool r_sta;

    r_sta = false;

    // cord the perform_match_old.
    std::memcpy(&perform_match_old, &perform_match, sizeof(MATCH_PERFORMANCE));

    // mapexsit: pure mapping.
    if(slam_enable == false)
    {
        char lvl_swin = Lvl_HighReso;

        // get MOri_matxz, MOri_kf2lsr.
        SWin_Set_node_s(lvl_swin, match_continuous_hmp, NULL);

        // get MCor_matxz.
        /// AlMatch_BruteForce(match_normal_hmap, lvl_swin, H_akey_node.lmmapchart_deal)    : valid.
        /// AlMatch_GSearch(match_normal_hmap, H_akey_node.lmmapchart_deal)                 : invalid.
        /// AlMatch_GN_XYClimbing(match_continuous_hmp, 75, H_akey_node.lmmapchart_deal)    : valid.
        if(AlMatch_GN_XYClimbing(match_continuous_hmp, 100, NULL))
        {
            almatch_srvpackdata->Msta_m2odom_tfrefresh2(true, MCor_matxz);
            lsrdta_deal->fresh_state(MCor_matxz);

            almatch_srvpackdata->Msta_source2robot = almatch_srvpackdata->Msta_m2odom *
                                                     almatch_srvtf.TFM_Odom2Baselink_R.returndata();

            r_sta = true;
        }

        SWin_ClearCord(lvl_swin);
    }

    // slam mode: mapping based on key frame.
    else
    {
        char lvl_swin = Lvl_HighReso;

        // get MOri_matxz, MOri_kf2lsr.
        SWin_Set_node_s(lvl_swin, match_continuous_slam, H_akey_node.lmmapchart_deal);

        // get MCor_matxz.
        /// AlMatch_BruteForce(match_normal_slam, lvl_swin, H_akey_node.lmmapchart_deal)    : valid.
        /// AlMatch_GSearch(match_normal_slam, H_akey_node.lmmapchart_deal)                 : invalid.
        /// AlMatch_GN_XYClimbing(match_continuous_slam, 75, H_akey_node.lmmapchart_deal)   : valid.
        if(AlMatch_GN_XYClimbing(match_continuous_slam, 100, H_akey_node.lmmapchart_deal))
        {
            r_sta = true;
        }

        SWin_ClearCord(lvl_swin);
    }

    return r_sta;
}

void Al_Match::SWin_Set_node_s(char lvl_swin, char mode, m_lmmapchart *lmmap)
{
    static RM_NORM IMat;

    node_count = 1;

    if((mode < mapg_slam_bondary))
    {   // when map exist
        MOri_odom = lsrdta_deal->matsta_o2lsr;
        MOri_matxz = almatch_srvpackdata->Msta_m2odom;

        MOri_kf2lsr = lsrdta_deal->mapexist_m2lsr;
    }
    else{
        MOri_odom = IMat.tmatrix;
        MOri_matxz = lsrdta_deal->slam_kf2lsr;

        MOri_kf2lsr = lsrdta_deal->slam_kf2lsr;
    }



    MCor_matxz = MOri_matxz;

    node_s[lvl_swin].node.data()[0] = SWin_hsize_x[lvl_swin];
    node_s[lvl_swin].node.data()[1] = SWin_hsize_y[lvl_swin];
    node_s[lvl_swin].node.data()[2] = SWin_hsize_yaw[lvl_swin];

    static vector<vtype_lsr>::iterator it_lsrdta;
    it_lsrdta = lsrdta_deal->lsrdta.begin();
    node_s[lvl_swin].values = mtype_lsrmatch(lvl_swin,
                                        mode,
                                        *almatch_srvmap,
                                        MOri_kf2lsr,
                                        lmmap,
                                        lsrdta_deal->isfull(),
                                        lsrdta_deal->lsrdta_size,
                                        it_lsrdta);

    node_best[lvl_swin].values = node_s[lvl_swin].values;
    node_best[lvl_swin].node = node_s[lvl_swin].node;
    node_best[lvl_swin].gpart = gpart_temp;

    static int nid_xy, nid_xyyaw;
    nid_xy = node_s[lvl_swin].node.data()[1] * SWin_size_x[lvl_swin] + node_s[lvl_swin].node.data()[0];
    nid_xyyaw = node_s[lvl_swin].node.data()[2] * SWin_size_xy[lvl_swin] + nid_xy;

    SWin_varea[lvl_swin][nid_xyyaw] = node_best[lvl_swin].values;
    SWin_checkedq[lvl_swin].push_back(nid_xyyaw);
}

void Al_Match::SWin_ClearCord(char lvl_swin)
{
    static vector<int>::iterator it;

    //double time_s;
    //boost::timer t_caculate;

    for(it=SWin_checkedq[lvl_swin].begin(); it!=SWin_checkedq[lvl_swin].end(); it++)
    {
        SWin_varea[lvl_swin][*it] = v_default;
    }

    SWin_checkedq[lvl_swin].clear();

    //time_s = t_caculate.elapsed();
    //ROS_INFO("[VMap_ReInit]: %lf", time_s); // totally 20ms
}

/// ***************************************************************************************************
//* I am a line, please don't erase me!!!! ************************************************************
/// ***************************************************************************************************

bool Al_Match::AlMatch_LoopMapping(void)
{
    static bool r_sta;

    r_sta = false;

    // cord the perform_match_old.
    std::memcpy(&perform_match_old, &perform_match, sizeof(MATCH_PERFORMANCE));

    {
        char lvl_swin = Lvl_LowReso; // Lvl_HighReso; // Lvl_LowReso;

        // get MOri_matxz, MOri_kf2lsr.
        SWin_Set_node_s(lvl_swin, match_normal_slam, H_akey_node.lmmapchart_deal);

        // get MCor_matxz.
        /// AlMatch_BruteForce(match_normal_slam, lvl_swin, H_akey_node.lmmapchart_deal)    : valid.
        if(AlMatch_BruteForce(match_normal_slam, lvl_swin, H_akey_node.lmmapchart_deal))
        {
            r_sta = true;
        }

        /*
        if(AlMatch_GN_XYClimbing(match_continuous_slam, 125, H_akey_node.lmmapchart_deal))
        {
            r_sta = true;
        }*/

        SWin_ClearCord(lvl_swin);
    }

    return r_sta;
}

bool Al_Match::AlMatch_LoopDeal(Eigen::Matrix4d &akey_old_source2lsr,
                                Eigen::Matrix4d &akey_source2lsr)
{
    /// mapping between nodes, and add new edge to the graph.  (akey_node > akey_node_old)
    static bool loop_exists; loop_exists = true;
    static vector<unit_edge>::iterator it;

    static Eigen::Matrix4d akey_old_kf2lsr, akey_kf2lsr;

    /// always add edge.
    /* check if edge is exists.
    for(it=H_akey_node_old.edges.begin();it!=H_akey_node_old.edges.end();it++)
    {
        if((*it).tnode_id == H_akey_node.node_id)
            loop_exists = false;
    }*/

    if(almatch_srvgraph->akey_node_old != almatch_srvgraph->akey_node) // (loop_exists)
    {
        /// step1. get message.
        static Eigen::Matrix4d mat_akey2akey_old;

        {
            akey_old_kf2lsr = H_akey_node_old.mat_source2lsr.inverse() * akey_old_source2lsr;
            akey_kf2lsr     = H_akey_node.mat_source2lsr.inverse() * akey_source2lsr;

            mat_akey2akey_old = akey_kf2lsr * akey_old_kf2lsr.inverse();

            // perform_match_old;
            // perform_match;
        }

        /// step2. add edge.
        {
            /* positive side.
            unit_edge fill_edge1;
            fill_edge1.edge_type = edge_loop;

            fill_edge1.mes_cov = perform_match_old.MCov + perform_match.MCov;
            fill_edge1.mes_data = mat_akey2akey_old;
            fill_edge1.fnode_id = almatch_srvgraph->akey_node;
            fill_edge1.tnode_id = almatch_srvgraph->akey_node_old;*/

            // negative side.
            unit_edge fill_edge2;
            fill_edge2.edge_type = edge_loop;

            fill_edge2.mes_cov = perform_match_old.MCov + perform_match.MCov;
            fill_edge2.mes_data = mat_akey2akey_old.inverse();
            fill_edge2.fnode_id = almatch_srvgraph->akey_node_old;
            fill_edge2.tnode_id = almatch_srvgraph->akey_node;

            //H_akey_node.edges.push_back(fill_edge1);
            H_akey_node_old.edges.push_back(fill_edge2);
        }

        /// step3. loop correction!
        {
            /*
            almatch_srvgraph->graph_find_path(almatch_srvgraph->akey_node,
                                              almatch_srvgraph->akey_node_old);
                                              */

            // slam_stop = true;
        }
    }

    /*
    static Eigen::Matrix4d mat_d;
    mat_d = almatch_srvgraph->akey_node->mat_source2lsr * MCor_matxz;

    ROS_INFO("I AM A LINE: LOOK DOWN");
    cout << almatch_srvgraph->akey_node_old->mat_source2lsr << endl;
    cout << endl;
    cout << mat_d << endl;
    cout << endl;
    ROS_INFO("I AM A LINE: LOOK UP");
    */

    //return loop_exists;

    return true;
}









