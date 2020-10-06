#ifndef __buildmap_H
#define __buildmap_H

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

    // TF Part ***********************************************/
    #include "tf/transform_broadcaster.h"
    #include "tf/transform_listener.h"

    // Message Part ******************************************/
    #include "std_msgs/String.h"
    #include "geometry_msgs/Pose.h"
    #include "geometry_msgs/PoseWithCovarianceStamped.h"
    #include "geometry_msgs/Twist.h"
    #include "geometry_msgs/Point.h"
    #include "nav_msgs/OccupancyGrid.h"
    #include "sensor_msgs/LaserScan.h"
    #include "visualization_msgs/Marker.h"
    #include "nav_msgs/Path.h"
    #include "sensor_msgs/JointState.h"
    #include "nav_msgs/Odometry.h"

    // Math Part *********************************************/
    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    // SELF DEFINE PART **************************************/
    #include "graph_io/graph_io.h"
    #include "optimizer/graph_opt1.h"

    #include "func_buildmap.h"


#endif




