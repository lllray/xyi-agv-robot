#include "al_match.h"

#include "../class_tool/hfunc.h"

#include "search_window_strategy.h"
#include "func_lsr_mapping.h"

bool slam_stop = false;
enum rebuild_mode{rm_nofresh = 0x00, rm_justfresh = 0x01, rm_newnode = 0x02};

// *************************************************************************************************
// > SWin Part
void Al_Match::Almatch_PramInit(float *param)
{
    SWin_XYRange[0] = param[0],
     SWin_XYRange[1] = param[1],
      SWin_XYreso[0] = param[2],    SWin_XYreso_2[0] = param[2] * 0.5f;
       SWin_XYreso[1] = param[3],    SWin_XYreso_2[1] = param[3] * 0.5f;

    SWin_YawRange[0] = param[4],
     SWin_YawRange[1] = param[5],
     SWin_YawReso[0] = param[6],   SWin_YawReso_2[0] = param[6] * 0.5f;
      SWin_YawReso[1] = param[7],   SWin_YawReso_2[1] = param[7] * 0.5f;

    H_square_sigma = param[8] * param[8];

    for(char lvl=0; lvl<2; lvl++)
    {
        SWin_hsize_x[lvl] = int(SWin_XYRange[lvl]/SWin_XYreso[lvl]);
        SWin_hsize_y[lvl] = int(SWin_XYRange[lvl]/SWin_XYreso[lvl]);
        SWin_hsize_yaw[lvl] = std::ceil(SWin_YawRange[lvl]/SWin_YawReso[lvl]);

        SWin_size_x[lvl] = (SWin_hsize_x[lvl] * 2 + 1);
        SWin_size_y[lvl] = (SWin_hsize_y[lvl] * 2 + 1);
        SWin_size_yaw[lvl] = (SWin_hsize_yaw[lvl] * 2 + 1);

        SWin_size_xy[lvl] = SWin_size_x[lvl] * SWin_size_y[lvl];
        SWin_size[lvl] = SWin_size_xy[lvl] * SWin_size_yaw[lvl];

        SWin_checkedq[lvl].clear();
        SWin_varea[lvl].resize(SWin_size[lvl], v_default);
    }

    Almatch_Paraminit = true;
}

/// *****************************************************************************
/// MappingPort

extern float gpart_temp;    /// defined in chart_mapping.cpp

static void norm_matrix4d(Eigen::Matrix4d &mat_in)
{
    mat_in.setZero();
    mat_in.data()[0] = 1.0f, mat_in.data()[5] = 1.0f, mat_in.data()[10] = 1.0f, mat_in.data()[15] = 1.0f;
}

bool Al_Match::AlMatch_MappingPort(char mapping_mode,
                                   CTool_tfm &tool_tfm,
                                   SLAM_DATA &srv_packdata,
                                   Struct_Map &srv_map,
                                   Model_Laser &srv_lsrmodel,
                                   list_graph &srv_graph,
                                   sensor_msgs::LaserScanConstPtr &lsr_data)
{
    #define ifdebugsteps_show   0

    if(Almatch_Paraminit)
    {
        double time_s;
        boost::timer t_caculate;
        static bool sta;

        #if ifdebugsteps_show == 1
        cout << "step 0 ";
        #endif

        // **********************************************************************************
        /// copy the services.
        almatch_srvmap =        &srv_map;
        almatch_srvpackdata =   &srv_packdata;
        std::memcpy(&almatch_srvtf, &tool_tfm, sizeof(CTool_tfm));
        almatch_srvgraph =      &srv_graph;

        #if ifdebugsteps_show == 1
        cout << "step 1 ";
        #endif

        // **********************************************************************************
        /// get lsr_filter data.

        if(lsrdta_deal == NULL)
            lsrdta_deal = new m_lsrdta;
        else
            lsrdta_deal->reset();

        #if ifdebugsteps_show == 1
        cout << "step 2 ";
        #endif

        #if ifdebugsteps_show == 1
        cout << "step a ";
        #endif

        sta = lsrdta_deal->filters(srv_lsrmodel, *almatch_srvpackdata, almatch_srvtf, lsr_data);

        #if ifdebugsteps_show == 1
        cout << "step b " << sta << " " ;
        #endif

        // **********************************************************************************
        static Eigen::Matrix4d mat_d_odom, mat_d_source2lsr, mat_d_source2lsr_old;

        // step1: considering the odometry.
        static bool dodom_first = true;
        static RM_NORM odom_old;
        mat_d_odom = odom_old.tmatrix.inverse() * lsrdta_deal->matsta_o2lsr;
        odom_old.tmatrix = lsrdta_deal->matsta_o2lsr;

        #if ifdebugsteps_show == 1
        cout << "step c " ;
        #endif

        if(dodom_first == true)
            dodom_first = false;
        else
            lsrdta_deal->slam_kf2lsr *= mat_d_odom;

        #if ifdebugsteps_show == 1
        cout << "step d " ;
        #endif

        if(sta)
        {
            #if ifdebugsteps_show == 1
            cout << "step e: " << int(mapping_mode) << " " ;
            #endif

            switch(mapping_mode)
            {
            case match_slaminit:
                almatch_srvgraph->graph_init(almatch_srvmap,
                                             lsrdta_deal,
                                             5.0f, 5.0f, DOUBLE_PI); // 51.2f, 51.2f, DOUBLE_PI); //
                break;

            case match_slammap:
                static bool enable_addlsr;
                static unsigned char rebuild_mmap = rm_nofresh;
                static bool ifloop_correction = false;

                enable_addlsr = true;

                #if ifdebugsteps_show == 1
                cout << "step 0 " ;
                #endif

                /// generate mapping chart.
                /*
                if(lmmapchart_deal == NULL)
                {   // step2: build local mapping map.
                    lmmapchart_deal = new m_lmmapchart;
                    lmmapchart_deal->buildmap(almatch_srvgraph->akey_node,
                                              almatch_srvmap, &H_akey_node.lsrdta);

                }else if(rebuild_mmap > rm_nofresh){
                    if(rebuild_mmap == rm_newnode)
                        lmmapchart_deal->reset();

                    lmmapchart_deal->buildmap(almatch_srvgraph->akey_node,
                                              almatch_srvmap, &H_akey_node.lsrdta);
                    //enable_addlsr = false;

                    rebuild_mmap = rm_nofresh;
                }
                */

                if(H_akey_node.lmmapchart_deal == NULL)
                {   // step2: build local mapping map.
                    H_akey_node.lmmapchart_deal = new m_lmmapchart;
                    H_akey_node.lmmapchart_deal->buildmap(almatch_srvgraph->akey_node,
                                                          almatch_srvmap, &H_akey_node.lsrdta);

                }else if(rebuild_mmap > rm_nofresh){
                    if(rebuild_mmap == rm_justfresh)
                        H_akey_node.lmmapchart_deal->buildmap(almatch_srvgraph->akey_node,
                                                              almatch_srvmap, &H_akey_node.lsrdta);
                    //enable_addlsr = false;

                    rebuild_mmap = rm_nofresh;
                }

                #if ifdebugsteps_show == 1
                cout << "step 1 " ;
                #endif

                // step3: mapping based on the key frame.
                static bool pub_message;
                static unsigned char sta_fresh_akeynode;

                pub_message = true;

                if(ifloop_correction)
                    sta = AlMatch_LoopMapping();
                else
                    sta = AlMatch_Mapping(true);

                if(sta == false) break; /// false: lsrdta_deal is unfull... wait next loop. hmm.. it may not work,,.

                #if ifdebugsteps_show == 1
                cout << "step 2 " ;
                #endif

                /// mapping message.
                mat_d_source2lsr_old = mat_d_source2lsr;
                mat_d_source2lsr = H_akey_node.mat_source2lsr * MCor_matxz;// *
                                   // (almatch_srvtf.TFM_Odom2Baselink_R.returndata().inverse() * tool_tfm.TFM_Odom2Baselink_R.returndata());;

                almatch_srvpackdata->Msta_source2robot = mat_d_source2lsr;
                almatch_srvpackdata->Msta_source2robot.data()[12] -= almatch_srvtf.TFM_Baselink2Laser_R.returndata().data()[12];
                almatch_srvpackdata->Msta_source2robot.data()[13] -= almatch_srvtf.TFM_Baselink2Laser_R.returndata().data()[13];

                // step5: loop correction.
                if(ifloop_correction)
                {
                    AlMatch_LoopDeal(mat_d_source2lsr_old,
                                     mat_d_source2lsr);

                    enable_addlsr = false;
                    ifloop_correction = false;
                }

                if(slam_stop){
                    ifloop_correction = false;
                    return false;
                }

                #if ifdebugsteps_show == 1
                cout << "step 3 " ;
                #endif

                {
                    // step4: check the akey node.
                    sta_fresh_akeynode = almatch_srvgraph->slam_fresh_akeynode(enable_addlsr,
                                                                               almatch_srvmap,
                                                                               lsrdta_deal,
                                                                               MCor_matxz,
                                                                               mat_d_source2lsr,
                                                                               perform_match);

                    #if ifdebugsteps_show == 1
                    cout << "step 4 " ;
                    #endif

                    if(sta_fresh_akeynode == S_oldknode)
                    {
                        lsrdta_deal->slam_kf2lsr = MCor_matxz;
                    }

                    else if(sta_fresh_akeynode == S_freshnode)
                    {
                        // step2: build local mapping map.
                        /// neeed build map
                        rebuild_mmap = rm_justfresh;

                        lsrdta_deal->slam_kf2lsr = MCor_matxz;
                    }

                    else if(sta_fresh_akeynode == S_change2newknode)
                    {
                        // step2: build local mapping map.
                        static RM_NORM Imat;
                        /// neeed build map
                        rebuild_mmap = rm_newnode;

                        lsrdta_deal->slam_kf2lsr = Imat.tmatrix;

                        pub_message = false;
                    }
                    else if((sta_fresh_akeynode == S_change2oldknode))
                    {
                        // step2: build local mapping map.
                        /// neeed build map
                        rebuild_mmap = rm_newnode;
                        ifloop_correction = true;

                        lsrdta_deal->slam_kf2lsr = H_akey_node.mat_source2lsr.inverse() * mat_d_source2lsr;

                    }

                    #if ifdebugsteps_show == 1
                    cout << "step 5 " ;
                    #endif

                    almatch_srvtf.tf_slam_brocasterFun(H_akey_node.mat_source2lsr,
                                                       lsrdta_deal->slam_kf2lsr);

                    // step6: pub tf, topics to rviz.
                    if(pub_message)
                    {
                        lsrdta_deal->Lsrdta_Pubcheck(*almatch_srvmap,
                                                     mat_d_source2lsr);

                        H_akey_node.lmmapchart_deal->nmap_pubcheck();
                    }

                }

                // step7: debug.

                break;

            case match_mapexist:

                // ROS_INFO(" DEBUG: STEP 1 ");

                AlMatch_Mapping(false);

                // ROS_INFO(" DEBUG: STEP 2 ");

                lsrdta_deal->Lsrdta_Pubcheck(*almatch_srvmap,
                                             lsrdta_deal->mapexist_m2lsr);

                // ROS_INFO(" DEBUG: STEP 3 ");

                break;

            default:
                break;
            }
        }


        #if ifdebugsteps_show == 1
        cout << "step endl; " << endl;
        #endif

        // if(sta)
        {
            almatch_srvpackdata->Msta_source2lsr = almatch_srvpackdata->Msta_source2robot * almatch_srvtf.TFM_Baselink2Laser_R.returndata();

            if(mapping_mode == match_mapexist)
            {
                /*
                static float d_syaw;
                d_syaw = atan2(almatch_srvpackdata->Msta_source2robot.data()[1],
                                almatch_srvpackdata->Msta_source2robot.data()[0]) * 57.3f;          // A1/A0



                cout << "[" <<  almatch_srvpackdata->Msta_source2robot.data()[12] << ", "
                            <<  almatch_srvpackdata->Msta_source2robot.data()[13] << ", "
                            <<  d_syaw << "], ";

                ROS_INFO("[Location]: %f, %f, %f. ", almatch_srvpackdata->Msta_source2robot.data()[12],
                                                     almatch_srvpackdata->Msta_source2robot.data()[13], d_syaw);    */

            }

            if(mapping_mode == match_slammap)
            {
                time_s = t_caculate.elapsed() * 1000;
                ROS_INFO("[AlMatch_MappingPort]: %.1lf ms", time_s);

                static double statis_t_max = 0;
                static unsigned int statis_t_ci = 0;
                static unsigned int statis_t_range1 = 0;    // 0~20ms
                static unsigned int statis_t_range2 = 0;    // 20~40ms
                static unsigned int statis_t_range3 = 0;    // 40~80ms
                static unsigned int statis_t_range4 = 0;    // 80~??ms

                ++statis_t_ci;

                if(time_s > statis_t_max)
                    statis_t_max = time_s;

                if(time_s <= 20)
                    ++statis_t_range1;
                else if (time_s <= 40)
                    ++statis_t_range2;
                else if (time_s <= 80)
                    ++statis_t_range3;
                else
                    ++statis_t_range4;

                cout << "TInfo: " << statis_t_max << endl <<
                        "     > " << float(statis_t_range1)/statis_t_ci << endl <<
                        "     > " << float(statis_t_range2)/statis_t_ci << endl <<
                        "     > " << float(statis_t_range3)/statis_t_ci << endl <<
                        "     > " << float(statis_t_range4)/statis_t_ci << endl;
            }
        }

        #if ifdebugsteps_show == 1
        cout << "sdeath; " << endl;
        #endif

        return true;
    }

    return false;
}





