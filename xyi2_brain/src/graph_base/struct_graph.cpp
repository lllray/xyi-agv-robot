/* ********************************************************************************
 * author: chenyingbing
 * time: 20170523   20:47   in XIAMEN University
 * illustration:
 *      pose graph optimization.
 *
 *      node.cpp edge.cpp
 *      :   construct the graph.
 *      <property> node_id.
 *
 *      table.cpp
 *      :   add the location construct
 *      <property> locate_id, locate_xy.
 *
 * *******************************************************************************/

#include "struct_graph.h"


/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

void list_graph::graph_topics_pub(void)
{
    static int i, j, size_node, size_edge, size_edge_sub1;
    static int tnode_id_c, tnode_id_ci;

    size_node = graph_nodes.size();

    for(i=0; i<size_node; i++)
    {
        cout << "[node " << graph_nodes[i].node_id << "]:" << endl;


        size_edge = graph_nodes[i].edges.size();
        size_edge_sub1 = size_edge - 1;

        if(size_edge > 0)
            tnode_id_c = graph_nodes[i].edges[0].tnode_id;;

        tnode_id_ci = 0;

        for(j=0; j<size_edge; j++)
        {
            //cout << graph_nodes[i].edges[j].mes_data << endl;
            //cout << graph_nodes[i].edges[j].mes_data.inverse() << endl << endl;

            if(graph_nodes[i].edges[j].tnode_id == tnode_id_c)
            {
                ++tnode_id_ci;

                if(j == size_edge_sub1)
                    cout << "(edges_to " << tnode_id_c << "[" << tnode_id_ci << "]" << ") > " << endl;
            }
            else
            {
                cout << "(edges_to " << tnode_id_c << "[" << tnode_id_ci << "]" << ") > " << endl;

                tnode_id_ci = 1;
                tnode_id_c = graph_nodes[i].edges[j].tnode_id;
            }

        }

        cout << endl;
    }
}


/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************


void list_graph::disable_addnew(void)
{
    enable_addnew = false;
}





