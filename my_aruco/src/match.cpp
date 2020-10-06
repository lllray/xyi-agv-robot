#include "match.h"

//The homogeneous transformation matrix of the coordinate of marker respect to the coordinate of world
float cmo[16] = 
{ 1,  0,  0,  0,
  1,  1,  0,  0,
  0,  0,  1,  0,
  0,  0,  0,  1};

Mat cmo_data(4,4,CV_32F,cmo);	
Mat xiaoyi_in_camera(4,1,CV_32F);
match::match()
{	
 	xiaoyi_in_camera.at<float>(0) = 0;
	xiaoyi_in_camera.at<float>(1) = 0;
	xiaoyi_in_camera.at<float>(2) = 0;
	xiaoyi_in_camera.at<float>(3) = 1;
}

/*warning:*remember to set the z in cam_localob_cmo to zero*/
Mat match::position_to_base_coordinate(Mat cam_localob_cmo)
{
	Mat cam_baseob_cmo(4,4,CV_32F);
	Mat xiaoyi_in_base(4,1,CV_32F);
 	Mat xiaoyi_in_base_2(3,1,CV_32F);	

	cam_baseob_cmo = cmo_data * cam_localob_cmo;
	xiaoyi_in_base = cam_baseob_cmo * xiaoyi_in_camera;
	
	xiaoyi_in_base_2.at<float>(0) = xiaoyi_in_base.at<float>(0);
	xiaoyi_in_base_2.at<float>(1) = xiaoyi_in_base.at<float>(1);
	xiaoyi_in_base_2.at<float>(2) = xiaoyi_in_base.at<float>(2);	


	Mat vpT(3,1,CV_32F);
		 
 	vpT.at<float>(0) = cam_baseob_cmo.at<float>(0,3);
 	vpT.at<float>(1) = cam_baseob_cmo.at<float>(1,3);	
 	vpT.at<float>(2) = cam_baseob_cmo.at<float>(2,3);	

	return vpT;

}

float match::direction_to_base_coordinate(Mat cam_localob_cmo)
{	
	Mat R(3,3,CV_32F);
	Mat direction(3,1,CV_32F);
	Mat x(3,1,CV_32F);
	Mat cam_baseob_cmo(4,4,CV_32F);
	float degree;

    cam_baseob_cmo = cmo_data * cam_localob_cmo;
	R.at<float>(0,0) = cam_baseob_cmo.at<float>(0,0);
	R.at<float>(0,1) = cam_baseob_cmo.at<float>(0,1);
	
	R.at<float>(0,2) = cam_baseob_cmo.at<float>(0,2);

	R.at<float>(1,0) = cam_baseob_cmo.at<float>(1,0);
	R.at<float>(1,1) = cam_baseob_cmo.at<float>(1,1);
	R.at<float>(1,2) = cam_baseob_cmo.at<float>(1,2);

	R.at<float>(2,0) = cam_baseob_cmo.at<float>(2,0);
	R.at<float>(2,1) = cam_baseob_cmo.at<float>(2,1);
	R.at<float>(2,2) = cam_baseob_cmo.at<float>(2,2);

	/*camera z axis vector*/
	direction.at<float>(0)=0;
	direction.at<float>(1)=0.01;
	direction.at<float>(2)=0;
	/*camera z axis vector expressed in base*/
	direction = R * direction;
	direction.at<float>(2)=0;//make direction down to the ob-xy plane
	x.at<float>(0)=0.01;
	x.at<float>(1)=0;
	x.at<float>(2)=0;
    degree= acos(x.dot(direction)/(sqrt(x.dot(x))*sqrt(direction.dot(direction))));

    if(direction.at<float>(0) > 0 && direction.at<float>(1) > 0)
    {
        if(direction.at<float>(1) > 0.001)
            return degree;
        else
            return 0.0;
    }
    else if(direction.at<float>(0) < 0 && direction.at<float>(1) > 0)
    {
        if(direction.at<float>(1) > 0.001)
            return degree;
        else
            return 3.14159265358979323846 ;
    }
		 
    else if(direction.at<float>(0) < 0 && direction.at<float>(1) <0)
    {
        if(direction.at<float>(1) < -0.001)
            return -degree;
        else
            return -3.14159265358979323846 ;
    }
	
 
    else if(direction.at<float>(0) > 0 && direction.at<float>(1) <0)
    {
        if(direction.at<float>(1) < -0.001)
            return -degree;
        else
            return 0.0;
    }
	else 
		return -1.0;
}
