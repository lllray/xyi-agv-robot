/* ********************************************************************************
 * author: chenyingbing
 * time: 20170628 15:10  in XIAMEN University
 * illustration:
 *      graph optimization.
 *
 * *******************************************************************************/

#include "graph_opt1.h"

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

extern FILE_HEAD file_head;
extern std::vector<FILE_NODE>  file_nodes;         // NODES PACK
extern std::vector<FILE_EDGE>  file_edges;         // EDGES PACK
extern FILE_TAIL file_tail;

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

static void protect_range_yaw(Eigen::Vector3f &vector)
{
    if(vector(2) > PI)
        vector(2) -= DOUBLE_PI;
    else if(vector(2) < -PI)
        vector(2) += DOUBLE_PI;
}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

bool graph_optimizer(void)
{
    /// check the data again.
    bool sta;   sta = true;

    sta &= (file_nodes.size() == file_head.num_nodes);
    sta &= (file_edges.size() == file_head.num_edges);

    if(!sta)
    {
        ROS_ERROR(" graph_optimizer: check fail.");
        return false;
    }

    /// allocate the deal data.
    #define K_EDGE_INFO  0.1f
    Eigen::Matrix3f mat3x3_zero;                mat3x3_zero.setZero();
    std::vector<Eigen::Vector3f> vectors_dX;    vectors_dX.resize(file_head.num_nodes, Eigen::Vector3f(0, 0, 0));
    std::vector<Eigen::Matrix3f> edges_info;    edges_info.resize(file_head.num_edges, mat3x3_zero);

    /// show the original data.
    /* ************************************************************
    struct FILE_NODE
        unsigned int node_id;               // 4 bytes
        Eigen::Vector3f X;                  // 12 bytes

        unsigned int num_edges;             // 4 bytes
        Eigen::Vector2i edges_id_ft;        // 8 bytes (from (0) to (1))

        unsigned int num_lsrdtas;           // 4 bytes
        Eigen::Vector2i lsrdtas_id_ft;      // 8 bytes (from (0) to (1))

    struct FILE_EDGE
        unsigned int edge_id;               // 4 bytes
        Eigen::Vector2i nodes_id_ft;        // 8 bytes

        Eigen::Vector3f C;                  // 12 bytes
        Eigen::Vector3f D;                  // 12 bytes
    ************************************************************ */

    int i, j;
    cout << "x x x x x x x x x x x x SHOW. ORIGINAL DATA x x x x x x x x x x x x" << endl << endl;

    for(i = 0; i<file_head.num_nodes; i++)
    {
        cout << "(NODE " << file_nodes[i].node_id <<  "): " <<
                file_nodes[i].X(0) << ", " << file_nodes[i].X(1) << ", " << file_nodes[i].X(2) << ";" << endl;

        for(j = file_nodes[i].edges_id_ft(0); j <= file_nodes[i].edges_id_ft(1); j++)
        {
            edges_info[j](0) = K_EDGE_INFO / file_edges[j].C(0);
            edges_info[j](4) = K_EDGE_INFO / file_edges[j].C(1);
            edges_info[j](8) = K_EDGE_INFO / file_edges[j].C(2);

            cout << "    + EDGE " << file_edges[j].edge_id << ": " <<
                    "(" << file_edges[j].nodes_id_ft(0) << ", " << file_edges[j].nodes_id_ft(1) << "): " <<
                    file_edges[j].D(0) << ", " << file_edges[j].D(1) << ", " << file_edges[j].D(2) << ";" << endl;
        }
    }
    cout << endl;

    /// solve the non-linear least square problem.
    Eigen::Vector2i nft_id;

    double cos_d, sin_d;
    double Dx_cos_d, Dx_sin_d, Dy_cos_d, Dy_sin_d;
    double dk, d1, d2, d3;
    Eigen::Vector3f vector_d, vector_residual;

    Eigen::Matrix3f Bmat, Imat, dImat;
    double step_alpha, step_alpha_yaw;
    Eigen::Vector3f hdm;

    int iterates_count = 0, iterates_show = 0;
    int iterates = 750 * file_head.num_edges, iterates_add = 1;
    double f_value;
    bool   fisrt_F_values = true;
    double F_values_oringin, F_values, F_values_o = 100;
    double F_highest = 0, F_Lowest = 100;

    Bmat.setZero(), Imat.setZero();
    Bmat.data()[0] = 1.0f, Bmat.data()[4] = 1.0f, Bmat.data()[8] = 2.0f;
    Imat.data()[0] = 1.0f, Imat.data()[4] = 1.0f, Imat.data()[8] = 1.5f;

    step_alpha = 1.0f, step_alpha_yaw = 1.25f;

    int d_iterates = iterates / 10;
    cout << "x x x x x x x x x x x x OPTIMIZING... x x x x x x x x x x x x" << endl << endl;
    cout << "iterates remained: " << endl;

    while(iterates--)
    {
        F_values = 0;

        for(i = 0; i<file_head.num_nodes; i++)
        {
            for(j = file_nodes[i].edges_id_ft(0); j <= file_nodes[i].edges_id_ft(1); j++)
            {
                nft_id = file_edges[j].nodes_id_ft;

                cos_d = std::cos(file_nodes[nft_id(0)].X(2));
                sin_d = std::sin(file_nodes[nft_id(0)].X(2));

                Dx_cos_d = file_edges[j].D(0) * cos_d;
                Dx_sin_d = file_edges[j].D(0) * sin_d;
                Dy_cos_d = file_edges[j].D(1) * cos_d;
                Dy_sin_d = file_edges[j].D(1) * sin_d;

                vector_d(0) = file_nodes[nft_id(0)].X(0) + Dx_cos_d - Dy_sin_d;
                vector_d(1) = file_nodes[nft_id(0)].X(1) + Dx_sin_d + Dy_cos_d;
                vector_d(2) = file_nodes[nft_id(0)].X(2) + file_edges[j].D(2);
                protect_range_yaw(vector_d);

                vector_residual = file_nodes[nft_id(1)].X - vector_d;
                protect_range_yaw(vector_residual);

                f_value = vector_residual.transpose() * edges_info[j] * vector_residual;
                F_values = F_values + f_value;

                dk = 1.0f;
                d1 = edges_info[j].data()[0] * vector_residual(0);
                d2 = edges_info[j].data()[4] * vector_residual(1);
                d3 = edges_info[j].data()[8] * vector_residual(2);

                if(nft_id(0) != 0)
                {
                    vectors_dX[nft_id(0)](0) -= (-d1 * dk);
                    vectors_dX[nft_id(0)](1) -= (-d2 * dk);

                    /*
                    vectors_dX[nft_id(0)](2) -= ( d1 * dk * ( -1 + Dx_sin_d + Dy_cos_d ) +
                                                  d2 * dk * ( -1 - Dx_cos_d + Dy_sin_d ) +
                                                  d3 * dk * ( -1 )
                                                );*/
                }

                if(nft_id(1) != 0)
                {
                    vectors_dX[nft_id(1)](0) -= d1 * dk,  // negative gradient direction.
                    vectors_dX[nft_id(1)](1) -= d2 * dk;
                    //vectors_dX[nft_id(1)](2) -= d3 * dk;
                }
            }
        }

        if(fisrt_F_values)
        {
            F_values_oringin = F_values;
            fisrt_F_values = false;
        }

        dImat.setZero();
        dImat.data()[0] = Imat.data()[0] * step_alpha;
        dImat.data()[4] = Imat.data()[4] * step_alpha;
        dImat.data()[8] = Imat.data()[8] * step_alpha_yaw;
        dImat = (Bmat + dImat).inverse();

        if( F_values <= F_values_o)
        {
            step_alpha = step_alpha * 0.95f;
            step_alpha_yaw = step_alpha_yaw * 0.95f;

            F_values_o = F_values;

            iterates -= iterates_add;
            //iterates_add += 1;  if(iterates_add >= file_head.num_edges) iterates_add = file_head.num_edges;
            if(iterates < 0) break;
        }else{
            step_alpha = step_alpha * 2.5f;             if(step_alpha > 250) step_alpha = 250;
            step_alpha_yaw = step_alpha_yaw * 2.5f;     if(step_alpha_yaw > 250) step_alpha_yaw = 250;
        }

        if( F_values > F_highest)
            F_highest = F_values;

        if( F_values < F_Lowest)
            F_Lowest = F_values;

        for(i = 1; i<file_head.num_nodes; i++)
        {
            hdm = dImat * vectors_dX[i];
            vectors_dX[i].setZero();

            file_nodes[i].X += hdm;
            protect_range_yaw(file_nodes[i].X);
        }

        ++iterates_count;

        ++iterates_show;
        if(iterates_show >= d_iterates)
        {
            iterates_show -= d_iterates;
            cout << "remained: " << iterates << "." << endl;
        }
    }

    cout << "x x x x x x x x x x x x SHOW. PERFORMANCE x x x x x x x x x x x x" << endl << endl;
    cout << "[ INFO ] ITERATES TIME: " << iterates_count << "." << endl <<
            "         ORIGIN  F_values: " << F_values_oringin << "." << endl <<
            "         FINAL   F_values: " << F_values << "." << endl <<
            "         HIGHEST F_values: " << F_highest << "." << endl <<
            "         LOWEST  F_values: " << F_Lowest << "." << endl;

    /// show modification.
    cout << endl << "x x x x x x x x x x x x SHOW. MODIFIED DATA x x x x x x x x x x x x" << endl << endl;
    for(i = 0; i<file_head.num_nodes; i++)
    {
        cout << "(NODE " << file_nodes[i].node_id <<  "): " <<
                file_nodes[i].X(0) << ", " << file_nodes[i].X(1) << ", " << file_nodes[i].X(2) << ";" << endl;
    }
    cout << endl;

    char sta_cin = 0;
    std::string yes_text = "yes", no_text = "no";
    std::string cin_text;

    int try_count = 3;

    while(try_count >= 0)
    {
        cout << "[ INFO ] IF ACCEPT THE GRAPH ? REPLY 'yes' or 'no': ";
        cin >> cin_text;

        if(cin_text == yes_text)
        {
            sta_cin = 1;
            ROS_INFO("ANS: ACCEPT THE RESULT.");
            break;
        }

        else if(cin_text == no_text)
        {
            sta_cin = 2;
            ROS_INFO("ANS: DROP THE RESULT.");
            break;
        }
        else
        {
            if(try_count == 0)
            {
                sta_cin = 3;
                ROS_ERROR("ANS: ABANDON BY DEFAULT FOR OUTRANGE THE LIMIT TO INPUT.");
                break;
            }
            else
                ROS_ERROR("ANS: INVALID INPUT. RETRY COUNT: %d.", try_count);

            try_count--;
        }

    }

    cout << endl;

    if(sta_cin == 1)
        return true;
    else
        return false;
}





