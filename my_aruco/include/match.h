#ifndef match_H
#define match_H

 
#include "opencv2/opencv.hpp"

/*vector saving the extrinsic matrix between child route mark  and father route mark */


using namespace cv;
class match
{

public:
match();

Mat position_to_base_coordinate( Mat cam_localob_cmo);
float direction_to_base_coordinate( Mat cam_localob_cmo);
};


#endif
