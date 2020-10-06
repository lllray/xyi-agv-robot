#ifndef __graph_opt1_H
#define __graph_opt1_H

    // Sys Part **********************************************/
    #include <ros/ros.h>

    #include <sstream>
    #include <iostream>
    #include <string>
    using namespace std;

    // C_Plus Standard Library *******************************/
    #include <algorithm>
    #include <signal.h>
    #include <vector>
    #include <boost/timer.hpp>

    #include "boost/thread/thread.hpp"
    #include "boost/thread/mutex.hpp"
    #include "boost/thread/recursive_mutex.hpp"

    // Math Part *********************************************/
    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "../graph_io/graph_io.h"

    #define PI          3.1415925f
    #define DOUBLE_PI   6.283185f

    bool graph_optimizer(void);

#endif
