#ifndef __r_controller_H
#define __r_controller_H

    #include <iostream>
    using namespace std;

    #include <ros/ros.h>
    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "../navi_config.h"

    #include <geometry_msgs/Twist.h>

    typedef class r_controller
    {
    public:
        r_controller()
        {
            state_v = 0,
            state_gyro = 0;
        }

        ~r_controller()
        {

        }

        void stop_control(void);

        void acelelimit_control(float control_T,
                                Eigen::Vector2f _target_vgyro);

        void instant_control(Eigen::Vector2f _target_vgyro);

        float state_v, state_gyro;

    private:

        void controlcmd_pub(float command_v, float command_gyro);

    }r_controller;

#endif
