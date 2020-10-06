#ifndef __al_match_H
#define __al_match_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "al_match_aidclass.h"

    #include "../class_tool/param_robot.h"

    #include "../class_tool/tfm.h"
    #include "../class_tool/slamdata.h"
    #include "../map/struct_map.h"

    #include "struct_localmapping_chart.h"
    #include "struct_lsr_filter.h"
    #include "../graph_base/struct_graph.h"

    #include "func_lsr_mapping.h"

    enum matching_mode{ match_slaminit = 0x00,
                         match_slammap = 0x01,
                        match_mapexist = 0x0F};

    typedef class Al_Match
    {
    public:
        Al_Match()
        {
            Almatch_Paraminit = false;

            lsrdta_deal = NULL;
            //lmmapchart_deal = NULL;
            lmmapchat_loop_deal = NULL;

            perform_match.reset();
        }

        ~Al_Match()
        {
            delete lsrdta_deal;
        }

        bool Almatch_Paraminit;
        void Almatch_PramInit(float *param);

        /// Functions. ---------------------------------------------------------------------------
        bool AlMatch_MappingPort(char mapping_mode,
                                 CTool_tfm &tool_tfm,
                                 SLAM_DATA &srv_packdata,
                                 Struct_Map &srv_map,
                                 Model_Laser &srv_lsrmodel,
                                 list_graph &srv_graph,
                                 sensor_msgs::LaserScanConstPtr &lsr_data);

    private:
        /// Handle Data. -------------------------------------------------------------------------
        //m_lmmapchart *lmmapchart_deal;
        m_lsrdta *lsrdta_deal;

        Struct_Map *almatch_srvmap;
        SLAM_DATA *almatch_srvpackdata;
        CTool_tfm almatch_srvtf;
        list_graph *almatch_srvgraph;

        m_lmmapchart *lmmapchat_loop_deal;

        MATCH_PERFORMANCE perform_match, perform_match_old;

        /// Param Data. ---------------------------------------------------------------------------
        float H_square_sigma;

        /// SWin Data. ----------------------------------------------------------------------------
        #define v_default       10
        #define v_default_2     8

        float SWin_XYRange[2],
               SWin_XYreso[2], SWin_XYreso_2[2],
              SWin_YawRange[2],
               SWin_YawReso[2], SWin_YawReso_2[2];

        int SWin_hsize_x[2], SWin_hsize_y[2], SWin_hsize_yaw[2];
        int SWin_size_x[2], SWin_size_y[2], SWin_size_yaw[2];
        int SWin_size_xy[2], SWin_size[2];

        std::vector<float> SWin_varea[2];
        std::vector<int> SWin_checkedq[2];

        int node_count;
        ALM_NODE node_s[2];
        ALM_NODE node_best[2];

        /// Functions. -------------------------------------------------------------------------
        Eigen::Matrix4d MOri_matxz;                 //
        Eigen::Matrix4d MOri_odom;                  // odom data before corection.
        Eigen::Matrix4d MOri_kf2lsr;                // matrix[key frame to lsr_link] before mapping.

        Eigen::Matrix4d MCor_matxz; //

        bool AlMatch_Mapping(bool slam_enable);

        void SWin_Set_node_s(char lvl_swin, char mode, m_lmmapchart *lmmap);
        void SWin_ClearCord(char lvl_swin);

        bool AlMatch_LoopMapping(void);
        bool AlMatch_LoopDeal(Eigen::Matrix4d &akey_old_source2lsr,
                              Eigen::Matrix4d &akey_source2lsr);

        /// > 1
        bool AlMatch_BruteForce(char mode, char lvl_swin, m_lmmapchart *lmmap);

        /// > 2
        int iterate_max;
        int iterate_count;

        /*
        bool AlMatch_GSearch(char mode, m_lmmapchart *lmmap);
        void func_gsearch_recursively(char lvl_swin,
                                      char mode,
                                      m_lmmapchart *lmmap,
                                      Eigen::Vector3i &node_now);*/

        /// > 3
        bool AlMatch_GN_XYClimbing(char mode, int iterates_t, m_lmmapchart *lmmap);

        /// >>--------------------------------------------------------->

        #define H_akey_node_old   almatch_srvgraph->graph_nodes[almatch_srvgraph->akey_node_old]
        #define H_akey_node       almatch_srvgraph->graph_nodes[almatch_srvgraph->akey_node]

    }Al_Match;

#endif











