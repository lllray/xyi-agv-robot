#ifndef __struct_lsr_filter_H
#define __struct_lsr_filter_H

    #include <iostream>
    using namespace std;

    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "sensor_msgs/LaserScan.h"

    #include "../class_tool/tfm.h"
    #include "../map/struct_map.h"
    #include "../class_tool/slamdata.h"
    #include "../sensors/model_laser.h"

    typedef class vtype_lsr
    {
    public:
        vtype_lsr()
        {
            isstrong = true;
        }

        ~vtype_lsr()
        {

        }

        bool isstrong;
        Eigen::Vector2f lsr_fxy;
    }vtype_lsr;

    typedef class mi_lsrdta
    {
    public:
        mi_lsrdta()
        {
            isfull = 0;
            lsrdta_size = 0;

            dtas.clear();
        }

        ~mi_lsrdta()
        {

        }

        bool isfull;
        float lsrmax_range;
        int lsrdta_size;

        float lreso;
        float _lreso;
        unsigned int llength, lsize;
        Eigen::Vector2f map_origin;
        Eigen::Vector2f map_xiuz;

        std::vector<Eigen::Vector2i> dtas;

    }mi_lsrdta;

    typedef class m_lsrdta
    {
    public:
        m_lsrdta() : enable_debug(false)
        {
            sta_full = false;
            lsrdta_size = 0;
            lsrdta.clear();

            slam_kf2lsr.setZero();
            slam_kf2lsr.data()[0] = 1.0f, slam_kf2lsr.data()[5] = 1.0f, slam_kf2lsr.data()[10] = 1.0f, slam_kf2lsr.data()[15] = 1.0f;
        }

        ~m_lsrdta()
        {

        }

        // ***************************************************************************
        // lsr data part.
        /// state
        Eigen::Matrix4d matsta_o2lsr;        // matrix odom to lsr (const).

        Eigen::Matrix4d mapexist_m2odom;     // matrix map to odom   correct after mapping.
        Eigen::Matrix4d mapexist_m2lsr;      // matrix map to lsr.

        Eigen::Matrix4d slam_kf2lsr;         // matrix keyframe to lsr.

        /// param
        float lsrmax_range;
        int lsrdta_size;
        std::vector<vtype_lsr> lsrdta;

        /// port function
        bool filters(Model_Laser &srv_lsrmodel,
                     SLAM_DATA &slampack,
                     CTool_tfm &tool_tfm,
                     sensor_msgs::LaserScanConstPtr &lsr_in);

        bool isfull(void);
        void fresh_state(Eigen::Matrix4d &m2odom);
        void reset(void);

        void Lsrdta_Pubcheck(Struct_Map &srv_map,
                             Matrix4d &mat_m2lsr);

    private:
        // ***************************************************************************
        // lsr data part.

        bool sta_full;

        const bool enable_debug;
        sensor_msgs::LaserScan lsr_show;

    }m_lsrdta;

    typedef class beam_lsrdta
    {
    public:
        beam_lsrdta() : beam_size(360), beam_size_2(180), default_disline(0.0f)
        {
            isfull = false;

            reset();
        }

        ~beam_lsrdta()
        {

        }

        const int   beam_size, beam_size_2;
        const float default_disline;

        bool isfull;
        float beam_range[360];

        void reset(void);
        void fill(Model_Laser &srv_lsrmodel,
                  sensor_msgs::LaserScanConstPtr &lsr_in);

    private:

    }beam_lsrdta;

#endif






