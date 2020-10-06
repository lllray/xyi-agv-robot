#ifndef __pkg_include_H
#define __pkg_include_H

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

    // Image Part ********************************************/
    #include <cv.h>
    #include <cv_bridge/cv_bridge.h>
    #include <image_transport/image_transport.h>
    //#include <opencv2/imgproc/imgproc.hpp>
    //#include <opencv2/highgui/highgui.hpp>
    #include <sensor_msgs/image_encodings.h>
    #include <image_transport/image_transport.h>
    #include <interactive_markers/interactive_marker_server.h>



    // ROS-PCL Part ******************************************/
    #include <pcl_ros/point_cloud.h>

    #include <pcl/point_types.h>
    #include "pcl/kdtree/kdtree_flann.h"

    #include <pcl_conversions/pcl_conversions.h>

    #include <sensor_msgs/PointCloud2.h>


#endif
