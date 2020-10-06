
#ifndef my_aruco_H
#define my_aruco_H

#include <opencv2/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/calib3d.hpp>  
#include <opencv2/aruco.hpp>
#include <iostream>
 
#include <iostream>
#include <cstdlib>
 
#include "ippe.h"
#include "match.h"
#include <unistd.h>


using namespace std;
using namespace cv;



class my_aruco{
public:
    my_aruco();
    void aruco_init();
    int detect();
    Mat coordinate_transform();
    int detect(Mat image);
    bool estimatePose(float minerrorRatio = 4 );
    bool  coordinate_transform(Mat &R, Mat &t );
    bool add_coordinate(Mat &image);

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    void detectorParams_init();
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    void cameraParams_init();
    vector< vector< Point2f > > corners;
    vector< int > ids;
    float markerLength;
    Mat rvec ;
    Mat tvec ;
    vector<cv::Point3f> object_points;

private:

};
#endif
