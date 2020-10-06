/* ********************************************************************************
 * author: chenyingbing
 * time: 20171102   14:07   in XIAMEN University
 * illustration:
 *      dynamic window method.
 *
 * *******************************************************************************/

#include "dwindow.h"

////////////////////////////////////////////////////////////////////////////////////

static void v3d_correct_yaw(Eigen::Vector3d &vpose)
{
    if(vpose(2) > NAVI_PI) vpose(2) -= NAVI_DOUBLE_PI;
    else if(vpose(2) < -NAVI_PI) vpose(2) += NAVI_DOUBLE_PI;
}

////////////////////////////////////////////////////////////////////////////////////
/// Port Functions

void dwindow::get_robot_shape(Eigen::Vector2d *_rcorner)
{
    static float dx, dy;

    dx = -param_robot->Robot_Origin[0], dy = -param_robot->Robot_Origin[1];
    _rcorner[0](0) = (*mat_robot).data()[0] * dx + (*mat_robot).data()[4] * dy + (*mat_robot).data()[12];
    _rcorner[0](1) = (*mat_robot).data()[1] * dx + (*mat_robot).data()[5] * dy + (*mat_robot).data()[13];

    dx = param_robot->Robot_Width - param_robot->Robot_Origin[0], dy = -param_robot->Robot_Origin[1];
    _rcorner[1](0) = (*mat_robot).data()[0] * dx + (*mat_robot).data()[4] * dy + (*mat_robot).data()[12];
    _rcorner[1](1) = (*mat_robot).data()[1] * dx + (*mat_robot).data()[5] * dy + (*mat_robot).data()[13];

    dx = param_robot->Robot_Width - param_robot->Robot_Origin[0], dy = param_robot->Robot_Height - param_robot->Robot_Origin[1];
    _rcorner[2](0) = (*mat_robot).data()[0] * dx + (*mat_robot).data()[4] * dy + (*mat_robot).data()[12];
    _rcorner[2](1) = (*mat_robot).data()[1] * dx + (*mat_robot).data()[5] * dy + (*mat_robot).data()[13];

    dx = -param_robot->Robot_Origin[0], dy = param_robot->Robot_Height - param_robot->Robot_Origin[1];
    _rcorner[3](0) = (*mat_robot).data()[0] * dx + (*mat_robot).data()[4] * dy + (*mat_robot).data()[12];
    _rcorner[3](1) = (*mat_robot).data()[1] * dx + (*mat_robot).data()[5] * dy + (*mat_robot).data()[13];

}

void dwindow::fresh_state(PARAM_ROBOT &_param_robot, Eigen::Matrix4d &_mat_robot,
                          Struct_Map &_srv_map, local_obsmap &_localmap,
                          float _state_v, float _state_gyro)
{
    param_robot = &_param_robot;
    mat_robot = &_mat_robot;
    srv_map = &_srv_map;
    localmap = &_localmap;
    state_v = _state_v;
    state_gyro = _state_gyro;

    _Matrix4dGetXYYaw(_mat_robot, v3d_robot);

    if(asearch_map_lvl == localobsmap_lvl)
    {
        map_reso = _srv_map.lreso[asearch_map_lvl];                 inverse_map_reso = 1.0f / map_reso;
        map_width = _srv_map.lwidth[asearch_map_lvl];
        map_height = _srv_map.lheight[asearch_map_lvl];
        map_size = _srv_map.lsize[asearch_map_lvl];
        map_xiuz(0) = _srv_map.map_xiuz[asearch_map_lvl].position.x,
        map_xiuz(1) = _srv_map.map_xiuz[asearch_map_lvl].position.y;
    }else
        ROS_ERROR("ERROR DWINDOW PARAM.");

}

bool dwindow::if_keepstate(void)
{
    #define check_t     2.5f
    static std::vector<Eigen::Vector3d> gpath_nodes;
    static float get_safevalue;
    static float s_percentage;

    generate_path(check_t, state_v, state_gyro, gpath_nodes);
    get_safevalue = get_path_safevalue(gpath_nodes, s_percentage);

    if(get_safevalue > mdissafe_boundary)  return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////////
/// Private Functions

// calculate the minest distance from the robot body to obstacle.
float dwindow::calculate_v3dpose_safevaule(Eigen::Matrix4d &_mat_robotpose)
{
    /* ***************************************************************
     rcorner[2]       rcorner[1]
                HHHHH                   H: area occupied by robot.      denoted by param_robot->Robot_Width & Robot_Height.
                HHMHH                   M: robot center:                denoted by param_robot->Robot_Origin.
                HHHHH
     rcorner[3]       rcorner[0]
    *************************************************************** */
    #define safe_value_sample_dis  0.125f

    static float dx, dy;
    static Eigen::Vector2d rcorner[4];  // denotes the corners' position of robot.

    static int   ix, iy;
    static float bx, by;

    static int i, j;
    static Eigen::Vector2d rdeal[2], rrobot;
    static Eigen::Vector3i v3i_deal;

    static float safevalue, dsafevalue;

    dx = -param_robot->Robot_Origin[0], dy = -param_robot->Robot_Origin[1];
    rcorner[0](0) = _mat_robotpose.data()[0] * dx + _mat_robotpose.data()[4] * dy + _mat_robotpose.data()[12];
    rcorner[0](1) = _mat_robotpose.data()[1] * dx + _mat_robotpose.data()[5] * dy + _mat_robotpose.data()[13];

    dx = param_robot->Robot_Width - param_robot->Robot_Origin[0], dy = -param_robot->Robot_Origin[1];
    rcorner[1](0) = _mat_robotpose.data()[0] * dx + _mat_robotpose.data()[4] * dy + _mat_robotpose.data()[12];
    rcorner[1](1) = _mat_robotpose.data()[1] * dx + _mat_robotpose.data()[5] * dy + _mat_robotpose.data()[13];

    dx = param_robot->Robot_Width - param_robot->Robot_Origin[0], dy = param_robot->Robot_Height - param_robot->Robot_Origin[1];
    rcorner[2](0) = _mat_robotpose.data()[0] * dx + _mat_robotpose.data()[4] * dy + _mat_robotpose.data()[12];
    rcorner[2](1) = _mat_robotpose.data()[1] * dx + _mat_robotpose.data()[5] * dy + _mat_robotpose.data()[13];

    dx = -param_robot->Robot_Origin[0], dy = param_robot->Robot_Height - param_robot->Robot_Origin[1];
    rcorner[3](0) = _mat_robotpose.data()[0] * dx + _mat_robotpose.data()[4] * dy + _mat_robotpose.data()[12];
    rcorner[3](1) = _mat_robotpose.data()[1] * dx + _mat_robotpose.data()[5] * dy + _mat_robotpose.data()[13];

    ix = std::ceil(param_robot->Robot_Width / safe_value_sample_dis ); bx = 1.0f / ix;
    iy = std::ceil(param_robot->Robot_Height / safe_value_sample_dis ); by = 1.0f / iy;

    safevalue = dwindow_abosolutely_safe;

    for(i=0; i<=ix; i++)
    {
        rdeal[0] = (rcorner[1] - rcorner[0])* i * bx + rcorner[0];
        rdeal[1] = (rcorner[2] - rcorner[3])* i * bx + rcorner[3];

        for(j=0; j<=iy; j++)
        {
            rrobot = (rdeal[1] - rdeal[0]) * j * by + rdeal[0];

            v3i_deal(0) = std::floor((rrobot(0) + map_xiuz(0)) * inverse_map_reso);
            v3i_deal(1) = std::floor((rrobot(1) + map_xiuz(1)) * inverse_map_reso);

            if ( ((v3i_deal(0)>=0)&&(v3i_deal(0)<map_width)) &&
                 ((v3i_deal(1)>=0)&&(v3i_deal(1)<map_height))
               )
            {
                v3i_deal(2) = v3i_deal(1) * map_width + v3i_deal(0);

                dsafevalue = std::min(srv_map->mdismap[asearch_map_lvl][v3i_deal(2)],
                                       localmap->mdismap[v3i_deal(2)]) ;

                if(dsafevalue < safevalue)
                    safevalue = dsafevalue;

            }
            // cout << " [" << rrobot(0) << ", " << rrobot(1) << "]";
        }
        // cout << endl;
    }

    return safevalue;
}

// _percentage: the proportion of safe part occupy whole path.
// if first node is unsafe: _snode_id = -1;
float dwindow::get_path_safevalue(std::vector<Eigen::Vector3d> &_pathnodes, float &_percentage)
{
    static int i, _size;
    static rmat mat_get;
    static float min_safevalue;

    _size = _pathnodes.size();
    _percentage = 0.0f;
    if(_size == 0) return dwindow_abosolutely_safe;

    min_safevalue = dwindow_abosolutely_safe;
    for(i=0; i<_size; i++)
    {
        _XyyawGetMatrix4d(_pathnodes[i], mat_get.mat);

        min_safevalue = std::min(min_safevalue, calculate_v3dpose_safevaule(mat_get.mat));

        if(calculate_v3dpose_safevaule(mat_get.mat) < mdissafe_boundary)
             break;
    }

    _percentage = float(i) / _size;

    return min_safevalue;
}

bool dwindow::get_path_info(std::vector<Eigen::Vector3d> &_pathnodes, Eigen::Vector3d &gnode,
                            float &_min_safevalue, float &_f_safevalue,
                            float &_f_dis2goal, float &_f_dir2goal)
{
    static int i, _size, _size_1;
    static rmat mat_get;
    static Eigen::Vector3d dnode;

    _size = _pathnodes.size();
    _size_1 = _size - 1;

    _min_safevalue = dwindow_abosolutely_safe;
    _f_dis2goal = INFINITY;
    _f_dir2goal = NAVI_PI;

    if(_size > 0)
    {
        _min_safevalue = dwindow_abosolutely_safe;
        for(i=0; i<_size; i++)
        {
            _XyyawGetMatrix4d(_pathnodes[i], mat_get.mat);

            _min_safevalue = std::min(_min_safevalue, calculate_v3dpose_safevaule(mat_get.mat));

            if(calculate_v3dpose_safevaule(mat_get.mat) < mdissafe_boundary)
                 break;
        }

        _XyyawGetMatrix4d(_pathnodes[_size_1], mat_get.mat);
        _f_safevalue = calculate_v3dpose_safevaule(mat_get.mat);

        dnode = gnode - _pathnodes[_size_1];
        _f_dis2goal = std::abs(dnode(0)) + std::abs(dnode(1)); // manhaton distance

        _f_dir2goal = std::abs( _pathnodes[_size_1](2) -
                                std::atan2(gnode(1) - _pathnodes[_size_1](1), gnode(0) - _pathnodes[_size_1](0)) );

    }else
        return false;

    return true;
}

// _pathnodes does not include the original.
void dwindow::generate_path(float limit_t, float _v, float _gyro,
                            std::vector<Eigen::Vector3d> &_pathnodes)
{
    #define pathsample_min_t    0.2f         // s
    #define pathsample_min_dis  0.25f        // m
    #define pathsample_min_angle 0.1745f     // radian

    static char run_mode;
    static bool is_v_0, is_gyro_0;
    static int rotate_direction;

    static float rotate_r, rotate_perimeter;

    // ***************************************************************************

    _pathnodes.clear();

    if(_v < 0) return; // check _v must >= 0;

    run_mode = runmode_stop;
    is_v_0 = (_v == 0), is_gyro_0 = (_gyro == 0);

    if(is_v_0 && is_gyro_0)   return;

    if(is_v_0) run_mode = runmode_purerotating;
    else if (is_gyro_0) run_mode = runmode_pureforward;
    else if (_gyro > 0)  { run_mode = runmode_runleft, rotate_direction = 1; }
    else if (_gyro < 0)  { run_mode = runmode_runright, rotate_direction = -1; }

    // ***************************************************************************
    static float dis_limit, rotate_limit;

    rotate_r = std::abs(_v / _gyro);
    rotate_perimeter = NAVI_DOUBLE_PI * rotate_r;

    dis_limit = _v * limit_t;
    if(dis_limit > MAX_MEASURE_DIS) dis_limit = MAX_MEASURE_DIS;

    rotate_limit = std::abs(_gyro * limit_t);
    if(rotate_limit > MAX_MEASURE_ANGLE)
        rotate_limit = MAX_MEASURE_ANGLE;

    static int sample_dis_ic, sample_angle_ic;
    static float sample_dis, sample_angle;

    sample_dis = _v * pathsample_min_t;
    if(sample_dis > pathsample_min_dis) sample_dis = pathsample_min_dis;

    sample_angle = std::abs(_gyro * pathsample_min_t);
    if(sample_angle < pathsample_min_angle)
        sample_angle = pathsample_min_angle;

    static int i;
    static float dx, dy, dangle;
    static Eigen::Vector3d ppose_map;

    sample_dis_ic = std::ceil(dis_limit / sample_dis);
    sample_dis = dis_limit / sample_dis_ic;

    sample_angle_ic = std::ceil(rotate_limit / sample_angle);
    sample_angle = rotate_limit / sample_angle_ic;

    if(run_mode >= runmode_runleft)
    {
        static int chose_ic;
        static float sample_deal;

        if(sample_dis_ic > sample_angle_ic)
        {
            chose_ic = sample_dis_ic;
            sample_deal = NAVI_DOUBLE_PI * sample_dis / rotate_perimeter;

        }else{
            chose_ic = sample_angle_ic;
            sample_deal = sample_angle;
        }

        for(i=1; i<=chose_ic; i++)
        {
            // get dx, dy, dangle.
            dangle = sample_deal * i;

            dx = m_sin(dangle) * rotate_r;
            dy = ( -m_cos(dangle) * rotate_r + rotate_r ) * rotate_direction;

            ppose_map(0) = mat_robot->data()[0] * dx + mat_robot->data()[4] * dy + mat_robot->data()[12];
            ppose_map(1) = mat_robot->data()[1] * dx + mat_robot->data()[5] * dy + mat_robot->data()[13];
            ppose_map(2) = v3d_robot(2) + dangle * rotate_direction;

            v3d_correct_yaw(ppose_map);
            _pathnodes.push_back(ppose_map);
        }

    }
    else if(run_mode == runmode_pureforward)
    {
        dy = 0;
        ppose_map(2) = v3d_robot(2);

        for(i=1; i<=sample_dis_ic; i++)
        {
            dx = i * sample_dis;

            ppose_map(0) = mat_robot->data()[0] * dx + mat_robot->data()[4] * dy + mat_robot->data()[12];
            ppose_map(1) = mat_robot->data()[1] * dx + mat_robot->data()[5] * dy + mat_robot->data()[13];

            _pathnodes.push_back(ppose_map);
        }
    }
    else if(run_mode == runmode_purerotating)
    {
        rotate_direction = (_gyro > 0 ? 1 : -1);

        dx = 0;
        dy = 0;
        ppose_map(0) = v3d_robot(0);
        ppose_map(1) = v3d_robot(1);

        for(i=1; i<=sample_angle_ic; i++)
        {
            dangle = sample_angle * i * rotate_direction;

            ppose_map(2) = v3d_robot(2) + dangle;

            v3d_correct_yaw(ppose_map);
            _pathnodes.push_back(ppose_map);
        }
    }
}




