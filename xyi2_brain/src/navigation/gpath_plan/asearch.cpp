/* ********************************************************************************
 * author: chenyingbing
 * time: 20171031   ??:??   in XIAMEN University
 * illustration:
 *      mange the asearch method.
 *
 * *******************************************************************************/

#include "asearch.h"

#define dange_range_num     6
static float dangerdist[dange_range_num] =   {0.0f, 0.1f, 0.25f, 0.5f, 0.75f, 1.0f};
static float danger_index[dange_range_num];

#define asearch_svalue_absolute_safe        0.0f
static float asearch_svalue_absolute_danger;

static Eigen::Vector3i p3i_parent_default(-1, -1, -1);
static Eigen::Vector4f values_fghs_default(0, 0, 0, 0);

////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////

void asearch_manage::get_map_p3ilocation(Eigen::Matrix4d &matpose_map, Eigen::Vector3i &v3i_out)
{
    v3i_out(0) = std::floor((matpose_map.data()[12] + map_xiuz.position.x) * inverse_map_reso);
    v3i_out(1) = std::floor((matpose_map.data()[13] + map_xiuz.position.y) * inverse_map_reso);
    v3i_out(2) = v3i_out(1) * map_width + v3i_out(0);
}

void asearch_manage::get_map_location(Eigen::Vector3i &v3i_in, Eigen::Vector2f &v2f_out)
{
    v2f_out(0) = v3i_in(0) * map_reso - map_xiuz.position.x;
    v2f_out(1) = v3i_in(1) * map_reso - map_xiuz.position.y;
}

void asearch_manage::asearch_init(float _robot_minssafedis,
                                  float _map_reso, int _map_width, int _map_height, int _map_size,
                                  geometry_msgs::Pose _map_xiuz)
{
    // alloc data
    static int w, h, l;
    if(map_size_c != _map_size)
    {
        map_size_c =  _map_size;

        // set parameter
        robot_minssafedis = _robot_minssafedis;

        map_reso = _map_reso,
         inverse_map_reso = 1.0f/map_reso;
        map_width = _map_width;
        map_height = _map_height;
        map_size = _map_size;
        map_xiuz.position.x = _map_xiuz.position.x,
            map_xiuz.position.y = _map_xiuz.position.y;

        /* **************************           x
         *    [0]      [1][0][7]                ^
         * [1]   [3]   [2]   [6]                |
         *    [2]      [3][4][5]        y <--- map_r
        ************************** */
        surlocate_nneighbours4[0] = 1,
         surlocate_nneighbours4[1] = map_width,
          surlocate_nneighbours4[2] = -1,
           surlocate_nneighbours4[3] = -map_width;

        surlocate_nneighbours8[0] = 1,              surlocate_nneighbours8[1] = 1 + map_width,
         surlocate_nneighbours8[2] = map_width,      surlocate_nneighbours8[3] = map_width - 1,
          surlocate_nneighbours8[4] = -1,             surlocate_nneighbours8[5] = -1 - map_width,
           surlocate_nneighbours8[6] = -map_width,     surlocate_nneighbours8[7] = -map_width + 1;

        step_g = map_reso;

        nodes_map.resize(map_size);

        for(l=0, w=0, h=0; l<map_size; l++, w++)
        {
            if(w >= map_width)
            {
                w -= map_width;
                h += 1;
            }

            asearch_node_reset(l);
            nodes_map[l].p3i_self(0) = w,
             nodes_map[l].p3i_self(1) = h,
              nodes_map[l].p3i_self(2) = l;
        }

        has_init = true;
    }

    std::vector<int>::iterator locate_travelled_nodes_it;
    for(locate_travelled_nodes_it = locate_travelled_nodes.begin(); locate_travelled_nodes_it != locate_travelled_nodes.end(); locate_travelled_nodes_it ++)
    {
        asearch_node_reset((*locate_travelled_nodes_it));
    }

    locate_travelled_nodes.clear();
    locate_openlist_fqueue.clear();

}

////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////

// //////////////////////////////////////////////////////////////////
void asearch_manage::asearch_node_reset(int locate_node)
{
    nodes_map[locate_node].node_sta = nsta_unchecked;
    nodes_map[locate_node].p3i_parent = p3i_parent_default;
    nodes_map[locate_node].values_fghs = values_fghs_default;

}

// //////////////////////////////////////////////////////////////////
float asearch_manage::asearch_calculate_h(Eigen::Vector3i &checknode)
{
    // manhattan distance
    return ((std::abs(checknode(0) - p3i_goal(0)) + std::abs(checknode(1) - p3i_goal(1))) * map_reso);
}

float asearch_manage::asearch_calculate_s(std::vector<float> &mdismap, Eigen::Vector3i &checknode)
{


    static int i;
    static float d, d_len;

    if(mdismap[checknode(2)] >= Disnode_Unkown)
        return asearch_svalue_absolute_safe;
    else if(mdismap[checknode(2)] <= robot_minssafedis)
        return asearch_svalue_absolute_danger;
    else{

        d = (mdismap[checknode(2)]-robot_minssafedis);

        for(i=0; i<dange_range_num; i++)
        {
            if(d < dangerdist[i]) break;
        }

        d_len = 1.0f/ (dangerdist[i] - dangerdist[i-1]);

        if(i == dange_range_num)  return asearch_svalue_absolute_safe;
        else{
            return ( (dangerdist[i] - d)*d_len*danger_index[i-1] + (d-dangerdist[i-1])*d_len*danger_index[i] );
        }
    }

}

// //////////////////////////////////////////////////////////////////

void asearch_manage::asearch_add2fqueue(asearch_node &add_node)
{
    int this_int_it;
    std::vector<int>::iterator this_it;
    bool is_insert = false;

    for(this_it = locate_openlist_fqueue.begin(), this_int_it = 0;
        this_it != locate_openlist_fqueue.end();
        this_it++, this_int_it++ )
    {
        if(add_node.values_fghs(0) >=  nodes_map[(*this_it)].values_fghs(0))
        {
            locate_openlist_fqueue.insert(locate_openlist_fqueue.begin()+this_int_it, add_node.p3i_self(2));
            is_insert = true;
            break;
        }
    }

    if(is_insert == false)
    {
        locate_openlist_fqueue.push_back(add_node.p3i_self(2));
    }

}

void asearch_manage::asearch_fqueue_rerange(asearch_node &change_node)
{
    int this_int_it;
    std::vector<int>::iterator this_it;
    bool is_change = false;

    for(this_it = locate_openlist_fqueue.begin(), this_int_it = 0;
        this_it != locate_openlist_fqueue.end();
        this_it++, this_int_it++ )
    {
        if(change_node.p3i_self(2) ==  nodes_map[(*this_it)].p3i_self(2))
        {
            locate_openlist_fqueue.erase(locate_openlist_fqueue.begin()+this_int_it);
            is_change = true;
            break;
        }
    }
    if(is_change)
        asearch_add2fqueue(change_node);
}

void asearch_manage::asearch_add2openlist(char search_mode,
                                          std::vector<float> &mdismap, nav_msgs::OccupancyGrid &lmap)
{
    static int surlocate_int_it;
    static std::vector<int>::iterator surlocate_it;
    static std::vector<int> *surlocate_nneighbours;
    static int locate_d;

    static float neighbour8_add;
    static float step_g_d;

    if( search_mode == smode_neighbour4)
    {
        surlocate_nneighbours = &surlocate_nneighbours4;

        neighbour8_add = 0.0f;
    }
    else if(search_mode == smode_neighbour8)
    {
        surlocate_nneighbours = &surlocate_nneighbours8;

        neighbour8_add = 0.414f;
    }
    else
        return;

    for(surlocate_it = surlocate_nneighbours->begin(), surlocate_int_it = 0;
        surlocate_it != surlocate_nneighbours->end(); surlocate_it++, surlocate_int_it++)
    {
        step_g_d = step_g * ( 1 + neighbour8_add*((surlocate_int_it%2) == 1) );

        locate_d = nodes_map[locate_node_active_now].p3i_self(2) + (*surlocate_it);

        if( (locate_d >= 0) && (locate_d < map_size) )
            if( (lmap.data[locate_d] != Mnode_Path) ||
                (nodes_map[locate_d].node_sta == nsta_closelist) ||
                (mdismap[locate_d] <= robot_minssafedis) )
                continue;
            else{
                if(nodes_map[locate_d].node_sta == nsta_openlist)
                {
                    if(nodes_map[locate_d].values_fghs(1) < (nodes_map[locate_node_active_now].values_fghs(1)-step_g_d))
                    {
                        nodes_map[locate_node_active_now].p3i_parent = nodes_map[locate_d].p3i_self;

                        nodes_map[locate_node_active_now].values_fghs(1) = nodes_map[locate_d].values_fghs(1) + step_g_d;
                        nodes_map[locate_node_active_now].values_fghs(0) = nodes_map[locate_node_active_now].values_fghs(1) +
                                                                            nodes_map[locate_node_active_now].values_fghs(2) +
                                                                             nodes_map[locate_node_active_now].values_fghs(3);

                        asearch_fqueue_rerange(nodes_map[locate_node_active_now]);
                    }
                }
                else if(nodes_map[locate_d].node_sta == nsta_unchecked)
                {
                    // add to travelled queue.
                    locate_travelled_nodes.push_back(locate_d);

                    // set infomation.
                    nodes_map[locate_d].node_sta = nsta_openlist;
                    nodes_map[locate_d].p3i_parent = nodes_map[locate_node_active_now].p3i_self;

                    nodes_map[locate_d].values_fghs(1) = nodes_map[locate_node_active_now].values_fghs(1) + step_g_d,
                     nodes_map[locate_d].values_fghs(2) = asearch_calculate_h(nodes_map[locate_d].p3i_self),
                      nodes_map[locate_d].values_fghs(3) = asearch_calculate_s(mdismap, nodes_map[locate_d].p3i_self);

                    nodes_map[locate_d].values_fghs(0) = nodes_map[locate_d].values_fghs(1) +
                                                          nodes_map[locate_d].values_fghs(2) +
                                                           nodes_map[locate_d].values_fghs(3);
                    // add to fqueue.
                    asearch_add2fqueue(nodes_map[locate_d]);

                }else
                    ROS_ERROR(" WRONG1 IN ASEARCH.CPP.");
            }
        else
            continue;
    }
}

void asearch_manage::asearch_set_node_active(void)
{
    locate_node_active_now = locate_openlist_fqueue.back();
    nodes_map[locate_node_active_now].node_sta = nsta_closelist;

    locate_openlist_fqueue.pop_back();
}

asearch_result asearch_manage::asearch_if_exist(void)
{
    if(nodes_map[p3i_goal(2)].node_sta != nsta_unchecked)
        return asearch_r_success;

    if(locate_openlist_fqueue.size() == 0)
        return asearch_r_runoff;

    return asearch_r_running;
}

// //////////////////////////////////////////////////////////////////
void asearch_manage::asearch_set_start(std::vector<float> &mdismap)
{
    /// if(nodes_map[p3i_start(2)].node_sta == nsta_unchecked)
    {
        // set travelled
        locate_travelled_nodes.push_back(p3i_start(2));

        // get infomation
        nodes_map[p3i_start(2)].values_fghs(1) = 0,
         nodes_map[p3i_start(2)].values_fghs(2) = asearch_calculate_h(p3i_start),

         asearch_svalue_absolute_danger = nodes_map[p3i_start(2)].values_fghs(2);
         danger_index[0] = asearch_svalue_absolute_danger,
         danger_index[1] = asearch_svalue_absolute_danger * 0.5f,
         danger_index[2] = asearch_svalue_absolute_danger * 0.3f,
         danger_index[3] = asearch_svalue_absolute_danger * 0.0f,
         danger_index[4] = asearch_svalue_absolute_danger * 0.0f,
         danger_index[5] = asearch_svalue_absolute_safe;

          nodes_map[p3i_start(2)].values_fghs(3) = asearch_calculate_s(mdismap, p3i_start);

        nodes_map[p3i_start(2)].values_fghs(0) = nodes_map[p3i_start(2)].values_fghs(1) +
                                                 nodes_map[p3i_start(2)].values_fghs(2) +
                                                 nodes_map[p3i_start(2)].values_fghs(3);



        // asearch_add2openlist
        // redundant...

        // asearch_set_node_active
        locate_node_active_now = p3i_start(2);
        nodes_map[locate_node_active_now].node_sta = nsta_closelist;

    }

}

////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////    ////////

bool asearch_manage::asearch_search(std::vector<float> &mdismap, nav_msgs::OccupancyGrid &lmap,
                                    Eigen::Matrix4d &mat_start, Eigen::Matrix4d &mat_goal)
{
    static bool des_sta;
    static asearch_node_mode smode = smode_neighbour4;
    static int asearch_r_cii;
    static asearch_result asearch_r_sta;

    if(has_init == true)
    {
        asearch_r_cii = 0;

        get_map_p3ilocation(mat_start, p3i_start);
        get_map_p3ilocation(mat_goal, p3i_goal);

        asearch_r_sta = asearch_r_wrongdes;
        des_sta = ( (lmap.data[p3i_goal(2)] == Mnode_Path) && (mdismap[p3i_goal(2)] >= robot_minssafedis) );

        if(des_sta)
        {
            asearch_set_start(mdismap);

            while(1)
            {
                ++ asearch_r_cii;

                asearch_add2openlist(smode, mdismap, lmap);

                asearch_r_sta = asearch_if_exist();
                if(asearch_r_sta != asearch_r_running)
                    break;

                asearch_set_node_active();
            }
        }

        // asearch_r_wrongdes = 0, asearch_r_success = 1, asearch_r_runoff = 2
        if(asearch_r_sta == asearch_r_wrongdes)
        {
            ROS_INFO("[NAVI]: INVALID DESTINATION");
            return false;
        }
        else if(asearch_r_sta == asearch_r_success)
        {
            ROS_INFO("[NAVI]: SUCCESS [LoopNum]: %d.", asearch_r_cii);
            return true;
        }
        else if(asearch_r_sta == asearch_r_runoff)
        {
            ROS_INFO("[NAVI]: FAIL TO SEARCH");
            return false;
        }
    }
    else
        return false;

    return true;
}

bool asearch_manage::asearch_extractpath(std::vector<Eigen::Vector2f> &vpath)
{
    // Eigen::Vector3i p3i_start, p3i_goal;
    static int locate_node_now;
    static Eigen::Vector2f path_node;

    locate_node_now = p3i_goal(2);
    vpath.clear();

    while(1)
    {
        get_map_location(nodes_map[locate_node_now].p3i_self, path_node);
        vpath.push_back(path_node);

        if(locate_node_now == p3i_start(2))
            return true;

        locate_node_now = nodes_map[locate_node_now].p3i_parent(2);
        if(locate_node_now == p3i_parent_default(2))
            return false;

    }
}

float asearch_manage::return_map_reso(void)
{
    return map_reso;
}

