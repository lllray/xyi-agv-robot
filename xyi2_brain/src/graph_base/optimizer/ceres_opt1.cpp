/* ********************************************************************************
 * author: chenyingbing
 * time: 20170622   09:53   in XIAMEN University
 * illustration:
 *  my first try to use ceres to solve the nonlinear least squares problem.
 *
 *  ceres: compile fail, unkown error!.
 *
 * *******************************************************************************/
#include "../struct_graph.h"

/*

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

static void XyyawGetMatrix4d(Eigen::Vector3d &In_xyyaw, Eigen::Matrix4d &Out_MatrixDta)
{
    static float d_cos, d_sin;

    d_sin = std::sin(In_xyyaw(2));
    d_cos = std::cos(In_xyyaw(2));

    Out_MatrixDta.setZero();

    Out_MatrixDta.data()[0] = d_cos;    Out_MatrixDta.data()[4] = -d_sin;                                       Out_MatrixDta.data()[12] = In_xyyaw(0);
    Out_MatrixDta.data()[1] = d_sin;    Out_MatrixDta.data()[5] = d_cos;                                        Out_MatrixDta.data()[13] = In_xyyaw(1);
                                                                            Out_MatrixDta.data()[10] = 1.0f;
                                                                                                                Out_MatrixDta.data()[15] = 1.0f;
}

static void Matrix4dGetXYYaw(Eigen::Matrix4d &In_MatrixDta, Eigen::Vector3d &Out_xyyaw)
{
    Out_xyyaw(0) = In_MatrixDta.data()[12];
    Out_xyyaw(1) = In_MatrixDta.data()[13];
    Out_xyyaw(2) = std::atan2(In_MatrixDta.data()[1],In_MatrixDta.data()[0]);
}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

static void protect_range_yaw(Eigen::Vector3d &vector)
{
    if(vector(2) > PI)
        vector(2) -= DOUBLE_PI;
    else if(vector(2) < -PI)
        vector(2) += DOUBLE_PI;
}


void list_graph::path_correction(void)
{
    double time_s;
    boost::timer t_caculate;

    /// *****************************************************************
    /// path edges.
    static int node_old, node_now;
    static int locate_now;
    static int tonext_edge_now, edge_mes;

    static bool path_sta; path_sta = true;

    static int node_num, edge_num;
    static std::vector<int> node_cord, edge_cord;
    static int edge_starttonext;

    static RM_NORM IMAT;
    static Eigen::Matrix3d path_edge_cov;
    static Eigen::Matrix4d path_edge_mes;

    node_old = node_id_g;
    node_now = node_id_g;
    locate_now = locate_goal;

    node_num = 0, edge_num = 0;
    node_cord.clear(), edge_cord.clear();

    path_edge_cov.setZero();
    path_edge_mes = IMAT.tmatrix;

    while(1)
    {
        tonext_edge_now = graph_table.grids[locate_now].AS_edge_cid;

        ++node_num;
        node_cord.push_back(node_now);

        if(locate_now == locate_start)
            break;

        node_old = node_now;
        node_now = graph_table.grids[locate_now].AS_parent_id;
        if(node_now == -1)
            break;

        locate_now = graph_nodes[node_now].locate_id;
        edge_mes = graph_nodes[node_now].edges[tonext_edge_now].tnode_id;

        ++edge_num;
        edge_starttonext = tonext_edge_now;
        edge_cord.push_back(tonext_edge_now);

        path_edge_cov = path_edge_cov + graph_nodes[node_now].edges[tonext_edge_now].mes_cov;
        path_edge_mes = graph_nodes[node_now].edges[tonext_edge_now].mes_data * path_edge_mes;

        if(edge_mes != node_old)
            path_sta = false;

        if(graph_table.grids[locate_now].AS_has_travelled == false)
        {
            path_sta = false;
            ROS_ERROR("<func> show_path: illeagal path.");
            break;
        }
    }

    /// *****************************************************************
    /// robust edge:
    static Eigen::Matrix3d robust_edge_cov;
    static Eigen::Matrix4d robust_edge_mes;
    static int robust_edge_id;

    static bool robust_edge_sta; robust_edge_sta = false;
    static vector<unit_edge>::iterator edge_it;

    // must be get by this way, because the edge is in path which not including robust edge.
    for(edge_it = graph_nodes[node_id_s].edges.begin(), robust_edge_id = 0;
        edge_it != graph_nodes[node_id_s].edges.end();
        edge_it++, robust_edge_id++)
    {
        if((*edge_it).tnode_id == node_id_g)
        {
            robust_edge_sta = true;
            robust_edge_cov = (*edge_it).mes_cov;
            robust_edge_mes = (*edge_it).mes_data;

            (*edge_it).edge_type = edge_normal; // turn to be normal edge.

            break;
        }
    }

    /// *****************************************************************
    /// correction:

    if((path_sta && robust_edge_sta) == false)
        ROS_INFO(" LOOP CORRECTION: PREPARATION FAIL.");
    else{
        ROS_INFO(" LOOP CORRECTION: PREPARATION STEP1 > SUCCESS.");

        /// data repreparation.
        static int i;
        static int this_node_id;
        static Eigen::Vector3d vector_d_source2lsr, vector_d_edge;
        static Eigen::Matrix4d mat_d_source2lsr;

        static std::vector<Eigen::Vector3d> opt_vertex;
        static std::vector<Eigen::Vector3d> opt_edge;

        static Eigen::Vector3d d_vertex_int; d_vertex_int.setZero();
        static std::vector<Eigen::Vector3d> opt_d_vertex;

        static std::vector<Eigen::Matrix3d> edges_info;


        // path edges.
        opt_d_vertex.clear();
        opt_vertex.clear(), opt_edge.clear();
        edges_info.clear();

        this_node_id = node_id_s;
        mat_d_source2lsr = graph_nodes[node_id_s].mat_source2lsr;
        Matrix4dGetXYYaw(mat_d_source2lsr, vector_d_source2lsr);

        // cout << "[ INFO ] A PATH: " << endl << "<node id: " << this_node_id << " >" << endl << mat_d_source2lsr << endl << endl;

        opt_vertex.push_back(vector_d_source2lsr);

        opt_d_vertex.push_back(d_vertex_int); // opt_d_vertex[0] == {0, 0, 0}

        for(i = (edge_num-1); i>=0; i--)
        {
            #define the_edge   graph_nodes[this_node_id].edges[edge_cord[i]]
            the_edge.edge_gcost += edge_gcost_add; // edge_cost correction.

            mat_d_source2lsr = mat_d_source2lsr * the_edge.mes_data;
            Matrix4dGetXYYaw(mat_d_source2lsr, vector_d_source2lsr);
            opt_vertex.push_back(vector_d_source2lsr);

            Matrix4dGetXYYaw(the_edge.mes_data, vector_d_edge);
            opt_edge.push_back(vector_d_edge);

            opt_d_vertex.push_back(d_vertex_int);

            edges_info.push_back(the_edge.mes_cov.inverse());

            // next node.
            this_node_id =  the_edge.tnode_id;

            // cout << "<node id: " << this_node_id << " >" << endl << mat_d_source2lsr << endl << endl;
        }

        // fusion edge.
        static double cos_d_const, sin_d_const;
        static double Dx_cos_d_const, Dx_sin_d_const, Dy_cos_d_const, Dy_sin_d_const;
        static Eigen::Vector3d vector_d1, vector_d2;
        static Eigen::Vector3d vector_fusion_edge, vector_d_const;
        static Eigen::Matrix3d fusion_edge_info, fusion_edge_cov;
        static Eigen::Matrix4d mat_fusion_edge;

        {
            fusion_edge_info = (robust_edge_cov.inverse() + path_edge_cov.inverse());
            fusion_edge_cov = fusion_edge_info.inverse();

            Matrix4dGetXYYaw(robust_edge_mes, vector_d1);
            Matrix4dGetXYYaw(path_edge_mes, vector_d2);

            vector_fusion_edge = fusion_edge_cov * (robust_edge_cov.inverse() * vector_d1 + path_edge_cov.inverse() * vector_d2);
            XyyawGetMatrix4d(vector_fusion_edge, mat_fusion_edge);

            cos_d_const = std::cos(opt_vertex[0](2));
            sin_d_const = std::sin(opt_vertex[0](2));

            Dx_cos_d_const = vector_fusion_edge(0) * cos_d_const;
            Dx_sin_d_const = vector_fusion_edge(0) * sin_d_const;
            Dy_cos_d_const = vector_fusion_edge(1) * cos_d_const;
            Dy_sin_d_const = vector_fusion_edge(1) * sin_d_const;

            vector_d_const(0) = Dx_cos_d_const - Dy_sin_d_const;
            vector_d_const(1) = Dx_sin_d_const + Dy_cos_d_const;
            vector_d_const(2) = vector_fusion_edge(2);
        }

        ROS_INFO(" LOOP CORRECTION: PREPARATION STEP2 > SUCCESS.");

        /// solution to the nonlinear least square problem.
        static int j, k, k_size, k_size_sub1;
        static int iterates_t, iterates_add_t;
        static double F_values, F_values_o;
        static double f_value;
        static Eigen::Vector3d vector_d, vector_residual;

        static Eigen::Matrix3d Bmat, Imat, dImat;
        static double step_alpha, step_alpha_yaw;
        static Eigen::Vector3d hdm;

        static double cos_d, sin_d;
        static double Dx_cos_d, Dx_sin_d, Dy_cos_d, Dy_sin_d;
        static double dk, d1, d2, d3;

        F_values_o = 100;
        iterates_t = edge_num * 30, iterates_add_t = 1;

        Bmat.setZero(), Imat.setZero();
        Bmat.data()[0] = 20.0f, Bmat.data()[4] = 20.0f, Bmat.data()[8] = 50.0f;
        Imat.data()[0] = 1.0f, Imat.data()[4] = 1.0f, Imat.data()[8] = 1.5f;

        step_alpha = 1.0f, step_alpha_yaw = 1.25f;

        // check if exists ERROR. But it is impossible to occur.
        if(opt_vertex.size() != (edge_num + 1))
            ROS_ERROR("<func> ceres_path_optimize: amazing: num_vertex != num_edge + 1");
        else
            k_size = (edge_num + 1);
            k_size_sub1 = edge_num;

            /// fusion edge.    node_id_s > node_id_g
            {
                opt_vertex[edge_num] = opt_vertex[0] + vector_d_const;  // fixed.
                protect_range_yaw(opt_vertex[edge_num]);
            }

            while(iterates_t--)
            {
                F_values = 0;

//                cout << "opt_vertex.push_back("
//                     << "Eigen::Vector3d(" << opt_vertex[0](0) << ", "
//                                           << opt_vertex[0](1) << ", "
//                                           << opt_vertex[0](2) << "));" << endl;

                // j: opt_edge;     edges_info;
                // k: opt_vertex;   opt_d_vertex;
                for(j = (edge_num-1), k = edge_num; j>=0; j--, k--)
                //for(j = 0, k = 1; j<=(edge_num-1); j++, k++)
                {
                    cos_d = std::cos(opt_vertex[k-1](2));
                    sin_d = std::sin(opt_vertex[k-1](2));

                    Dx_cos_d = opt_edge[j](0) * cos_d;
                    Dx_sin_d = opt_edge[j](0) * sin_d;
                    Dy_cos_d = opt_edge[j](1) * cos_d;
                    Dy_sin_d = opt_edge[j](1) * sin_d;

                    vector_d(0) = opt_vertex[k-1](0) + Dx_cos_d - Dy_sin_d;
                    vector_d(1) = opt_vertex[k-1](1) + Dx_sin_d + Dy_cos_d;
                    vector_d(2) = opt_vertex[k-1](2) + opt_edge[j](2);
                    protect_range_yaw(vector_d);

                    vector_residual = opt_vertex[k] - vector_d;
                    protect_range_yaw(vector_residual);

                    f_value = vector_residual.transpose() * edges_info[j] * vector_residual;
                    F_values = F_values + f_value;

                    dk = 1.0f;
                    d1 = edges_info[j].data()[0] * vector_residual(0);
                    d2 = edges_info[j].data()[4] * vector_residual(1);
                    d3 = edges_info[j].data()[8] * vector_residual(2);

                    dImat.data()[0] = Imat.data()[0] * step_alpha;
                    dImat.data()[4] = Imat.data()[4] * step_alpha;
                    dImat.data()[8] = Imat.data()[8] * step_alpha_yaw;
                    dImat = (Bmat + dImat).inverse();

                    ROS_ERROR("CERES_OPT1.CPP: Hmm... dImat is used in wrong way!.");

                    if(k != edge_num)
                    {
                        opt_d_vertex[k](0) -= d1 * dk,  // negative gradient direction.
                        opt_d_vertex[k](1) -= d2 * dk,
                        opt_d_vertex[k](2) -= d3 * dk;
                    }

                    if(k != 1)
                    {
                        opt_d_vertex[k-1](0) -= (-d1 * dk);
                        opt_d_vertex[k-1](1) -= (-d2 * dk);

                        opt_d_vertex[k-1](2) -= ( d1 * dk * ( -1 + Dx_sin_d + Dy_cos_d ) +
                                                  d2 * dk * ( -1 - Dx_cos_d + Dy_sin_d ) +
                                                  d3 * dk * ( -1 )
                                                );
                    }


//                    cout << "opt_edge.push_back("
//                         << "Eigen::Vector3d(" << opt_edge[j](0) << ", "
//                                               << opt_edge[j](1) << ", "
//                                               << opt_edge[j](2) << "));" << endl;

//                    cout << "edges_info[" << j << "]" << "<<" <<
//                            edges_info[j].data()[0] << ",0,0,0," << edges_info[j].data()[4] << ",0,0,0," << edges_info[j].data()[8] << ";" << endl;

//                    cout << "opt_vertex.push_back("
//                         << "Eigen::Vector3d(" << opt_vertex[k](0) << ", "
//                                               << opt_vertex[k](1) << ", "
//                                               << opt_vertex[k](2) << "));" << endl;

                }

                // revise the factors.
                if( F_values < F_values_o)
                {
                    step_alpha = step_alpha * 0.95f;
                    step_alpha_yaw = step_alpha_yaw * 0.95f;

                    F_values_o = F_values;
                }else{
                    iterates_t -= iterates_add_t;
                    iterates_add_t += 1;
                    if(iterates_t < 0) break;

                    step_alpha = step_alpha * 2.5f;
                    step_alpha_yaw = step_alpha_yaw * 2.5f;
                }

                for(k=1; k<k_size_sub1; k++)
                {
                    dImat.data()[0] = Imat.data()[0] * step_alpha;
                    dImat.data()[4] = Imat.data()[4] * step_alpha;
                    dImat.data()[8] = Imat.data()[8] * step_alpha_yaw;

                    hdm = (Bmat + dImat).inverse() * opt_d_vertex[k];
                    opt_d_vertex[k].setZero();

                    opt_vertex[k] += hdm;
                    protect_range_yaw(opt_vertex[k]);

                }

                // cout << "F: " << F_values << endl;
            }

        ROS_INFO(" LOOP CORRECTION ONCE: FValue %f.", F_values);

        /// final correction to data.
        static Eigen::Matrix4d mat_d_source2lsr_old;
        this_node_id = node_id_s;
        mat_d_source2lsr = graph_nodes[node_id_s].mat_source2lsr;
        mat_d_source2lsr_old = mat_d_source2lsr;

        // cout << "[ INFO ] B PATH: " << endl << "<node id: " << this_node_id << " >" << endl << mat_d_source2lsr << endl << endl;

        for(i = (edge_num-1), k = 1; i>=0; i--, k++)
        {
            #define the_edge   graph_nodes[this_node_id].edges[edge_cord[i]]
            the_edge.edge_gcost += edge_gcost_add; // edge_cost correction.

            XyyawGetMatrix4d(opt_vertex[k], mat_d_source2lsr);
            mat_d_source2lsr_old = mat_d_source2lsr;

            // next node.
            this_node_id =  the_edge.tnode_id;

            // cout << "<node id: " << this_node_id << " >" << endl << mat_d_source2lsr << endl << endl;

            // modify.
            {
                graph_nodes[this_node_id].mat_source2lsr = mat_d_source2lsr;
                the_edge.mes_data = mat_d_source2lsr_old.inverse() * mat_d_source2lsr;
            }
        }

        graph_nodes[node_id_s].edges[robust_edge_id].mes_data = mat_fusion_edge;
        graph_nodes[node_id_s].edges[robust_edge_id].mes_cov= fusion_edge_cov;

        time_s = t_caculate.elapsed();
        ROS_INFO(" LOOP CORRECTION FINISH: %lf s.", time_s);

    }
}
*/

