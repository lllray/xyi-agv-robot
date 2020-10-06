/* ********************************************************************************
 * author: chenyingbing
 * time: 20170523   20:47   in XIAMEN University
 * illustration:
 *  the basic procedure in slam.
 *
 * *******************************************************************************/
#include "struct_graph.h"

enum slam_sta{  slam_sta_normal,
                slam_sta_addnewkey
             };

unsigned char list_graph::slam_fresh_akeynode(bool enable_addlsr,
                                              Struct_Map *srv_map,
                                              m_lsrdta *mlsr_in,
                                              Eigen::Matrix4d &mat_kf2lsr,
                                              Eigen::Matrix4d &mat_source2lsr,
                                              MATCH_PERFORMANCE &perform_match)
{
    #define giveup_limit    0.1f
    static bool info_once;  info_once = true;

    static bool sta_location;
    static bool sta;
    static unsigned char r_sta;
    static int node_id, locate_id;
    static Eigen::Vector3i locate_xy;

    static ros::NodeHandle nstru;
    static ros::Publisher beepPublish = nstru.advertise<std_msgs::String>("RobotPort_BeepAck", 1);
    static std_msgs::String beepDta;

    locate_id = get_table_locateid(sta_location, mat_source2lsr, locate_xy);
    sta = graph_table.grids[locate_id].isfilled;

    node_id = graph_table.grids[locate_id].node_id;

    /// ***************************************************************************
    {   /// Message Print.

        if(enable_addnew)
            if(sta==false)
            {
                ROS_INFO("[%d] PLEASE STOP TO ADD NEW DATA.", perform_match.issteady);

                beepDta.data = "beep_ack";
                beepPublish.publish(beepDta);

                info_once = false;
            }

        //graph_topics_pub();
    }

    /// ***************************************************************************

    static bool enable_freshonce;   enable_freshonce = false;
    static bool f_dvailable;
    static Eigen::Matrix4d mat_d1, mat_d2, mat_d_kf2lsr;
    static bool ifcheck_fresh;

    {
        // step1: cord the old.
        akey_node_old = akey_node;

        // step2: get the situations.
        if(sta == true)
        {
            akey_node = node_id;

            ifcheck_fresh = true;

            if(akey_node != akey_node_old)
            {   /// TS_ToOld
                // see if see the "oldnode".
                mat_d1 = graph_nodes[akey_node_old].mat_source2lsr.inverse() * mat_source2lsr;
                mat_d2 = graph_nodes[akey_node_old].mat_source2lsr.inverse() * graph_nodes[akey_node].mat_source2lsr;
                f_dvailable = graph_nodes[akey_node_old].if_available(mat_d1, mat_d2);

                mat_d1 = graph_nodes[akey_node].mat_source2lsr.inverse() * mat_source2lsr;
                mat_d2 = graph_nodes[akey_node].mat_source2lsr.inverse() * graph_nodes[akey_node_old].mat_source2lsr;
                f_dvailable &= graph_nodes[akey_node].if_available(mat_d1, mat_d2);

                if(sta_location && f_dvailable)
                {
                    ifcheck_fresh = false;
                    r_sta = S_change2oldknode;
                }else{
                    akey_node = akey_node_old;
                    r_sta = S_oldknode;
                }
            }

            if(ifcheck_fresh)
            {
                /// TS_RemainedOld
                if( (graph_nodes[akey_node].num_subunit < graph_nodes[akey_node].nummax_subunit) &&
                    (perform_match.giveup_part >= giveup_limit)
                  )
                {
                    graph_nodes[akey_node].num_subunit += graph_nodes[akey_node].num_addadd;

                    enable_freshonce = true;
                    r_sta = S_freshnode;
                }
                else
                    r_sta = S_oldknode;
            }
        }
        else{
            /// TS_ToNew
            if(enable_addnew && perform_match.issteady)
            {
                /// add new vetex to the graph.
                // >fill unit_node
                // RM_NORM Imat;
                unit_node fill_node;
                fill_node.mat_source2lsr = mat_source2lsr;

                fill_node.node_id = vertexNum++;
                fill_node.locate_id = locate_id;
                fill_node.locate_xy = locate_xy;

                fill_node.lsrdta.isfull = false;

                fill_node.add_lsrdata_v3(graph_nodes[akey_node],
                                         srv_map,
                                         mat_kf2lsr, mlsr_in);

                graph_nodes.push_back(fill_node);

                // >fill unit_table
                table_mes fill_table;

                fill_table.isfilled = true;
                fill_table.node_id = fill_node.node_id;
                graph_table.grids[fill_node.locate_id] = fill_table;

                akey_node = (vertexNum - 1);

                node_id = fill_node.node_id;

                /// add new edge to the graph.  akey_node_old > akey_node
                // only positive side.
                unit_edge fill_edge1;
                fill_edge1.mes_cov = perform_match.MCov;
                fill_edge1.mes_data = mat_kf2lsr;
                fill_edge1.fnode_id = akey_node_old;
                fill_edge1.tnode_id = akey_node;

                graph_nodes[akey_node_old].edges.push_back(fill_edge1);

                enable_addlsr = false; // !!!
                r_sta = S_change2newknode;

            }
            else{
                enable_addlsr = false; // !!!

                akey_node = akey_node_old;
                r_sta = S_oldknode;
            }
        }

        // ROS_INFO("STATE: %d, %d.", sta_location, perform_match.issteady);
        // ROS_INFO("Enable1: %d, %d.", enable_addlsr, enable_freshonce);

        // step4: actions:
        if(enable_addnew && enable_addlsr)
        {
            mat_d_kf2lsr = graph_nodes[akey_node].mat_source2lsr.inverse() * mat_source2lsr;

            graph_nodes[akey_node].add_lsrdata_v2(enable_freshonce,
                                                  srv_map,
                                                  mat_d_kf2lsr,
                                                  mlsr_in);
        }

        // if(info_once)
        //     ROS_INFO("[%.0f%%] NORMAL RUNNING.", perform_match.giveup_part*100);

        //ROS_INFO("DEBUG: %d.", r_sta);

        return r_sta;
    }

}

