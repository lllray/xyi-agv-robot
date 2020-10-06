/* ********************************************************************************
 * author: chenyingbing
 * time: 20170627   10:44   in XIAMEN University
 * illustration:
 *      I/O between data and file.
 *
 * *******************************************************************************/

#include "graph_io.h"

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

static void Matrix4dGetXYYaw(Eigen::Matrix4d &In_MatrixDta, Eigen::Vector3f &Out_xyyaw)
{
    Out_xyyaw(0) = In_MatrixDta.data()[12];
    Out_xyyaw(1) = In_MatrixDta.data()[13];
    Out_xyyaw(2) = std::atan2(In_MatrixDta.data()[1],In_MatrixDta.data()[0]);
}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

void list_graph::graph_pack_data(std::string &catkin_route)
{
    char *pack;
    ROS_INFO(" GRAPH PACK DATA: START.");

    // HEAD PART.
    /// *** /// &&& /// lelele /// ### /// wowowo /// ))) /// @@@ /// !!! ///
    FILE_HEAD fhead;

    fhead.num_nodes = 0;
    fhead.num_edges = 0;
    fhead.num_grids = 0;

    // PARTS: NODES, EDGES, LSRDATA.
    /// *** /// &&& /// lelele /// ### /// wowowo /// ))) /// @@@ /// !!! ///
    FILE_NODE fnode;
    std::vector<FILE_NODE> file_nodes;   file_nodes.clear();                    // NODES PACK

    int record_node_id;
    int record_edges_id = 0;
    std::vector<unit_node>::iterator node_it;

    FILE_EDGE fedge;
    std::vector<FILE_EDGE> file_edges;   file_edges.clear();                    // EDGES PACK

    std::vector<unit_edge>::iterator edge_it;

    int record_nmapgrid_id = 0;
    std::vector<float> filenmaps_grid;       filenmaps_grid.clear();            // GRIDS PACK

    std::vector<float>::iterator grids_it;

    int debug_total_node_num = graph_nodes.size();

    for(node_it = graph_nodes.begin(), record_node_id = 0; node_it != graph_nodes.end(); node_it++, record_node_id++)
    {
        {
            fnode.node_id = record_node_id;
            Matrix4dGetXYYaw((*node_it).mat_source2lsr, fnode.X);

            fnode.num_edges = (*node_it).edges.size();
            fnode.edges_id_ft(0) = record_edges_id, fnode.edges_id_ft(1) = record_edges_id + (fnode.num_edges - 1);

            fnode.nmap_llength = (*node_it).lsrdta.llength;
            fnode.nmap_size = (*node_it).lsrdta.lsize;
            fnode.nmap_reso = (*node_it).lsrdta.lreso;
            fnode.nmap_xiuz = (*node_it).lsrdta.map_xiuz;

            fnode.nmap_gid_ft.data()[0] = record_nmapgrid_id,  fnode.nmap_gid_ft.data()[1] = record_nmapgrid_id + (fnode.nmap_size - 1);

            for(edge_it = (*node_it).edges.begin(); edge_it != (*node_it).edges.end(); edge_it++, record_edges_id++)
            {
                fedge.edge_id = record_edges_id;
                fedge.nodes_id_ft(0) = (*edge_it).fnode_id, fedge.nodes_id_ft(1) = (*edge_it).tnode_id;
                fedge.C(0) = (*edge_it).mes_cov(0), fedge.C(1) = (*edge_it).mes_cov(4), fedge.C(2) = (*edge_it).mes_cov(8);
                Matrix4dGetXYYaw((*edge_it).mes_data, fedge.D);

                file_edges.push_back(fedge);
            }

            for(grids_it = (*node_it).nmap.begin(); grids_it != (*node_it).nmap.end(); grids_it++, record_nmapgrid_id++)
            {
                filenmaps_grid.push_back((*grids_it));

                fhead.num_grids += 1;
            }

            file_nodes.push_back(fnode);
        }

        fhead.num_nodes += 1;
        fhead.num_edges += fnode.num_edges;

        ROS_INFO(" GRAPH PACKING: %d/%d.", (fnode.node_id+1), debug_total_node_num);
    }

    // TAIL PART.
    /// *** /// &&& /// lelele /// ### /// wowowo /// ))) /// @@@ /// !!! ///
    FILE_TAIL ftail;

    ROS_INFO(" GRAPH PACK DATA: STEP1 > DATA PREPARAION.");

    ///  >> FINAL STATISTICAL.
    /*  ******************************************************************************
    /// FILE_HEAD                       //          [Unit Type: FILE_HEAD]

    // > FILE_INTERVAL

    unsigned int  num_nodes;            // 4 bytes  [Unit Type: FILE_NODE]          yes
    unsigned int  locate2nodes;         // 4 bytes                                  yes

    // > FILE_INTERVAL

    unsigned int  num_edges;            // 4 bytes  [Unit Type: FILE_EDGE]          yes
    unsigned int  locate2edges;         // 4 bytes                                  yes

    // > FILE_INTERVAL

    unsigned int num_grids;             // 4 bytes  [Unit Type: float]              yes
    unsigned int locate2grids;          // 4 bytes                                  yes

    // > FILE_INTERVAL

    /// FILE_TAIL
    unsigned int  locate2tail;          // 4 bytes  [Unit Type: FILE_TAIL]
    ******************************************************************************  */
    FILE_INTERVAL   interval_normal;

    {
        unsigned int size_file_interval = sizeof(FILE_INTERVAL);

        //> fufill the message of FILEHEAD.
        fhead.locate2nodes = sizeof(FILE_HEAD) + size_file_interval;

        fhead.dtasize_nodes = sizeof(FILE_NODE)*fhead.num_nodes;

        fhead.locate2edges = fhead.locate2nodes + fhead.dtasize_nodes + size_file_interval;

        fhead.dtasize_edges = sizeof(FILE_EDGE)*fhead.num_edges;

        fhead.locate2grids = fhead.locate2edges + fhead.dtasize_edges + size_file_interval;

        fhead.dtasize_grids = sizeof(float)*fhead.num_grids;

        fhead.locate2tail = fhead.locate2grids + fhead.dtasize_grids + size_file_interval;

        fhead.dtasize_total = sizeof(FILE_HEAD) + size_file_interval +
                                fhead.dtasize_nodes + size_file_interval +
                                fhead.dtasize_edges + size_file_interval +
                                fhead.dtasize_grids + size_file_interval +
                              sizeof(FILE_TAIL);

        //> alloc the data.
        pack = (char *) std::malloc(fhead.dtasize_total);

        memcpy(pack + 0, &fhead, sizeof(FILE_HEAD));                                            // HEAD
        memcpy(pack + 0 + sizeof(FILE_HEAD), &interval_normal, size_file_interval);
        ROS_INFO(" GRAPH PACK DATA: HEAD READY");

        memcpy(pack + fhead.locate2nodes, file_nodes.data(), fhead.dtasize_nodes);              // NODES
        memcpy(pack + fhead.locate2nodes + fhead.dtasize_nodes, &interval_normal, size_file_interval);

        file_nodes.clear();
        ROS_INFO(" GRAPH PACK DATA: NODES READY");

        memcpy(pack + fhead.locate2edges, file_edges.data(), fhead.dtasize_edges);              // EDGES
        memcpy(pack + fhead.locate2edges + fhead.dtasize_edges, &interval_normal, size_file_interval);

        file_edges.clear();
        ROS_INFO(" GRAPH PACK DATA: EDGES READY");

        memcpy(pack + fhead.locate2grids, filenmaps_grid.data(), fhead.dtasize_grids);          // GRIDS
        memcpy(pack + fhead.locate2grids + fhead.dtasize_grids, &interval_normal, size_file_interval);

        filenmaps_grid.clear();
        ROS_INFO(" GRAPH PACK DATA: GRIDS READY");

        memcpy(pack + fhead.locate2tail, &ftail, sizeof(FILE_TAIL));                            // TAIL
        ROS_INFO(" GRAPH PACK DATA: TAIL READY");

        std::string save_file_name = "/slam_graph.data";
        catkin_route = catkin_route + save_file_name;

        {
            int write_number;
            int file_fd = open(catkin_route.data(),
                              O_CREAT |     // if file did not exist, create it.
                              O_WRONLY |    // write only.
                              O_TRUNC,      // if file exist, empty it.
                              00700);

            if(file_fd == -1)
               ROS_ERROR(" FILE OPEN FAIL.");

            write_number = write(file_fd, pack, fhead.dtasize_total);

            if(write_number != fhead.dtasize_total)
                ROS_ERROR(" FILE WRITE FAIL: WRITE NUMBER != FILE_TOTAL_SIZE");

            close(file_fd);
        }

        free(pack);

        ROS_INFO(" GRAPH PACK DATA: WRITE TO FILE, SIZE: %d.", fhead.dtasize_total);
        ROS_INFO(" FILE ROUTE: %s.", catkin_route.data());
        ROS_INFO(" PLASE PRESS CTRL + C TO EXIST THE PROGRAM. ");
    }
}







