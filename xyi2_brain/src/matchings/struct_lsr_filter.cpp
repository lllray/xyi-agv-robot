/* ********************************************************************************
 * author: chenyingbing
 * time: 20170512   14:55   in XIAMEN University
 * illustration:
 *      lsr_filter:
 *          i.  used for store the laser data: slam.
 *          ii. laser filter for more accurate mapping.
 *
 * *******************************************************************************/

#include "struct_lsr_filter.h"
#include "../class_tool/hfunc.h"

typedef class filter_core
{
public:
    filter_core() : filter_length(0.01f)
    {

    }
    ~filter_core(){}

    bool corefunc(int j, float angle, float arc_incre, float range)
    {
        /// ****************************************************************
        /// Angle_O,P0,P1 filter.
        static float arc_angle, arc_l, drange, drange_l;
        static float p;

        /// ****************************************************************
        /// Density filter.
        static float arc_length;

        if(j == 0)
        {
            angle_c = angle;
            range_c = range;

            arc_length_add = 0;
            grid_ci = 0;

            return false;
        }

        else{
            arc_angle = angle - angle_c;
            drange = range - range_c;
            range_c = range;
            angle_c = angle;

            arc_l = arc_angle * std::min(range, range_c);            
            drange_l = drange;

            arc_length = arc_incre * range_c;
            arc_length_add += arc_length;

            /// ****************************************************************
            /// Angle_O,P0,P1 filter.
            p = arc_l / drange_l;

            if(std::abs(p) < 0.364f) //0.577f)    // 90^o - 30^o = 60^o
                return false;


            /// ****************************************************************
            /// Density filter.
            if(arc_length_add < filter_length)
                return false;

        }

        ++grid_ci;
        arc_length_add = 0;

        return true;
    }

    int grid_num(void){ return grid_ci; }

private:

    float angle_c, range_c;

    const float filter_length;
    float arc_length_add;

    int grid_ci;

}filter_core;


static filter_core sub_filter;

bool m_lsrdta::filters(Model_Laser &srv_lsrmodel,
                       SLAM_DATA &slampack,
                       CTool_tfm &tool_tfm,
                       sensor_msgs::LaserScanConstPtr &lsr_in)
{
    bool sta;
    int i, j;
    float ang, range;

    Eigen::Vector2f f_xy;
    vtype_lsr push_dta;

    if((sta_full == false) &&
        sin_chart_init() && tool_tfm.TFM_Odom2Laser_R.isActive() && (lsr_in.use_count() > 0))
    {
        int d_size = lsr_in->ranges.size();

        {
            lsr_show.header.frame_id = "/base_laser_link_r";

            lsr_show.angle_min = lsr_in->angle_min;
            lsr_show.angle_max = lsr_in->angle_max;
            lsr_show.angle_increment = lsr_in->angle_increment;

            lsr_show.range_min = lsr_in->range_min;
            lsr_show.range_max = lsr_in->range_max;

            lsr_show.ranges.clear();
            lsr_show.ranges.resize(d_size, NAN);
        }

        lsrmax_range = srv_lsrmodel.range_max_95percent;

        /// the message used in the situation of map_exists .
        matsta_o2lsr = tool_tfm.TFM_Odom2Laser_R.returndata();

        mapexist_m2odom = slampack.Msta_m2odom;
        mapexist_m2lsr = mapexist_m2odom * matsta_o2lsr;

        for(i=0, j=0; i<d_size; i+=1)
        {
            // Ignores min range readings
            if(lsr_in->ranges[i] < 0.1f)
              continue;

            // Ignores max range readings
            if(lsr_in->ranges[i] >= srv_lsrmodel.range_max_95percent)
              continue;

            // Check for NaN
            if(lsr_in->ranges[i] != lsr_in->ranges[i])
              continue;

            ang = srv_lsrmodel.angle_range[0] + i*srv_lsrmodel.angle_increment;
            range = lsr_in->ranges[i];

            // Check for angle limit
            if ((ang > srv_lsrmodel.ANGLE_RANGE_LIMIT) || (ang < - srv_lsrmodel.ANGLE_RANGE_LIMIT))
                continue;

            f_xy(0) = range * m_cos(ang);
            f_xy(1) = range * m_sin(ang);

            sta = sub_filter.corefunc(j,
                                      ang,
                                      srv_lsrmodel.angle_increment,
                                      range);
            ++j;

            if(sta)
            {
                push_dta.lsr_fxy = f_xy;
                lsrdta.push_back(push_dta);

                {
                    lsr_show.ranges[i] = lsr_in->ranges[i];
                }   
            }
        }

        if(sub_filter.grid_num() > 0)
        {
            lsrdta_size = sub_filter.grid_num();

            sta_full = true;
        }

        return true;
    }

    /// DEBUG: Time cost 1 ~ 2ms;
    //int ddd = temp_lsrdta.size();
    //ROS_INFO("HELLO: %d, %d ", sub_filter.grid_num(), ddd);
    //ROS_INFO("HELLO: %d ", lsrdta_size);

    return false;
}

bool m_lsrdta::isfull(void)
{
    return sta_full;
}

void m_lsrdta::fresh_state(Eigen::Matrix4d &m2odom)
{
    mapexist_m2odom = m2odom;
    mapexist_m2lsr = mapexist_m2odom * matsta_o2lsr;
}

void m_lsrdta::reset(void)
{
    sta_full = false;
    lsrdta_size = 0;
    lsrdta.clear();
}

void m_lsrdta::Lsrdta_Pubcheck(Struct_Map &srv_map,
                               Matrix4d &mat_m2lsr)
{
    static const char T = 5;
    static char t = 0;

    /// lsrdta_test pub
    static ros::NodeHandle nstru;
    static ros::Publisher lsr_datapub = nstru.advertise<sensor_msgs::LaserScan>("RobotPort_Scan_S",1);
    lsr_show.header.stamp = ros::Time::now();
    lsr_datapub.publish(lsr_show);

    // this code only work near by the origin
    if(enable_debug && sta_full && srv_map.Map_alloc_sta())
    {
        ++t;

        if(t >= T)
        {
            /// lsrmap_test pub
            float x_d, y_d;
            float px, py;
            int x1, y1, lid1;
            nav_msgs::OccupancyGrid map_pub;

            map_pub.header.stamp = ros::Time::now();
            map_pub.header.frame_id = "/map_r";
            map_pub.info.resolution = srv_map.lreso[1];
            map_pub.info.width = srv_map.lwidth[1];
            map_pub.info.height = srv_map.lheight[1];
            map_pub.info.origin = srv_map.map_origin[1];

            map_pub.data.resize(srv_map.lsize[1]);

            int i;

            for(i=0; i<lsrdta_size; i++)
            {
                x_d = lsrdta[i].lsr_fxy(0);
                y_d = lsrdta[i].lsr_fxy(1);

                px = mat_m2lsr.data()[0] * x_d + mat_m2lsr.data()[4] * y_d + mat_m2lsr.data()[12];
                py = mat_m2lsr.data()[1] * x_d + mat_m2lsr.data()[5] * y_d + mat_m2lsr.data()[13];

                x1 = std::floor((px + srv_map.map_xiuz[1].position.x)/srv_map.lreso[1]);
                y1 = std::floor((py + srv_map.map_xiuz[1].position.y)/srv_map.lreso[1]);

                if ( ((x1>=0)&&(x1<srv_map.lwidth[1])) &&
                     ((y1>=0)&&(y1<srv_map.lheight[1]))
                   )
                {
                    lid1 = y1 * srv_map.lwidth[1] + x1;

                    map_pub.data[lid1] = 100;
                }
            }

            static ros::Publisher map_datapub = nstru.advertise<nav_msgs::OccupancyGrid>("lsrmap_test",1);
            map_datapub.publish(map_pub);

            t = 0;

            //ROS_INFO("MAP DBUG ");

        }
    }
}

void beam_lsrdta::reset(void)
{
    static int i;
    for(i=0; i<360; i++)
        beam_range[i] = -1.0f;  // < default_disline

    isfull = false;
}

void beam_lsrdta::fill(Model_Laser &srv_lsrmodel,
                       sensor_msgs::LaserScanConstPtr &lsr_in)
{
    static int i;
    static float ang, range;
    static int d_iang;

    static float ang_range;
    ang_range = srv_lsrmodel.angle_range[1] - srv_lsrmodel.angle_range[0];

    static int di, j;
    di = std::floor( (srv_lsrmodel.lsr_size / (ang_range*57.3f)) ) / 2;

    for(i=0, j=0; i<srv_lsrmodel.lsr_size; i+=di)
    {
        // Ignores min range readings
        if(lsr_in->ranges[i] < 0.1f)
          continue;

        // Ignores max range readings
        if(lsr_in->ranges[i] >= srv_lsrmodel.range_max_95percent)
          continue;

        // Check for NaN
        if(lsr_in->ranges[i] != lsr_in->ranges[i])
          continue;

        ang = srv_lsrmodel.angle_range[0] + i*srv_lsrmodel.angle_increment;
        range = lsr_in->ranges[i];

        d_iang = std::floor(ang * 57.3f + 180);

        if(beam_range[d_iang] < 0)
        {
            beam_range[d_iang] = range;
            //++j;
        }
        else if(range < beam_range[d_iang])
            beam_range[d_iang] = range;

    }

    isfull = true;

    // ROS_INFO("BEAM_LSRDTA FILL: %d ", j);
}



