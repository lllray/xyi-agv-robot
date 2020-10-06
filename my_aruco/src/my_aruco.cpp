#include "my_aruco.h"

	double distc[5];
	double camm[9];
 	Mat camMatrix(3,3,CV_64FC1,camm);
	Mat distCoeffs(1,5,CV_64FC1,distc);




void impl__aruco_getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T ) {

    	assert ( M.cols==M.rows && M.cols==4 );
    	assert ( M.type() ==CV_32F || M.type() ==CV_64F );
	//extract the rotation part
    	cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    	cv::SVD svd ( r33 );
    	cv::Mat Rpure=svd.u*svd.vt;
    	cv::Rodrigues ( Rpure,R );
    	T.create ( 3,1,M.type() );
    	if ( M.type() ==CV_32F )
        	for ( int i=0; i<3; i++ )
            		T.at<float> ( i,0)=M.at<float> ( i,3 );
    	else
        	for ( int i=0; i<3; i++ )
            	T.at<double> ( i,0)=M.at<double> ( i,3 );
}

my_aruco::my_aruco()
{
	dictionary =  cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);;
	detectorParams = cv::aruco::DetectorParameters::create();
	detectorParams_init();
	cameraParams_init();
    markerLength = 0.20;
	object_points = 
	{
	cv::Point3f(-markerLength/2,markerLength/2,0), 
	cv::Point3f(markerLength/2,markerLength/2,0),
	cv::Point3f(markerLength/2,-markerLength/2,0),
	cv::Point3f(-markerLength/2,-markerLength/2,0)
	};
 
}
void my_aruco::detectorParams_init()
{

	detectorParams->adaptiveThreshWinSizeMin= 20;
	detectorParams->adaptiveThreshWinSizeMax = 20;
	detectorParams->adaptiveThreshWinSizeStep = 10;
	detectorParams->adaptiveThreshConstant = 7;
	detectorParams->minMarkerPerimeterRate = 0.03;
	detectorParams->maxMarkerPerimeterRate = 4.0;
	detectorParams->polygonalApproxAccuracyRate = 0.05;
	detectorParams->minCornerDistanceRate =  0.05;
	detectorParams->minDistanceToBorder = 3;
	detectorParams->minMarkerDistanceRate = 0.05;
	detectorParams->cornerRefinementWinSize = 5;
	detectorParams->cornerRefinementMaxIterations = 30;
	detectorParams->cornerRefinementMinAccuracy = 0.1;
	detectorParams->markerBorderBits = 1;
	detectorParams->perspectiveRemovePixelPerCell = 8;
	detectorParams->perspectiveRemoveIgnoredMarginPerCell = 0.13;
	detectorParams->maxErroneousBitsInBorderRate = 0.35;
	detectorParams->minOtsuStdDev =  5.0;
    detectorParams->errorCorrectionRate = 0.6;
//    detectorParams->doCornerRefinement = true; // do corner refinement in markers

}
void my_aruco::cameraParams_init()
{
    camm[0]=1019.833690526177;
    camm[1]=0;
    camm[2]=339.5609579379868;
    camm[3]=0;
    camm[4]=1021.441129333163;
    camm[5]=252.6969760523888;
    camm[6]=0;
    camm[7]=0;
    camm[8]=1;

    distc[0]=-0.2432161200349792;
    distc[1]=-0.2194168675857364;
    distc[2]=0.004226653579144373;
    distc[3]=-0.001575804746741926;
    distc[4]=0;
}

int  my_aruco::detect(Mat image)
{

	vector< int > ids_;
	vector< vector< Point2f > > corners_;
	cv::aruco::detectMarkers(image, dictionary, corners_, ids_, detectorParams);

	ids = ids_;
	corners = corners_;
	return ids_.size();
}

bool my_aruco::estimatePose(float minerrorRatio){

	std::vector<cv::Point2f>  imgPoints;
	imgPoints = corners[0];
	
    	if ( rvec.empty()){//if no previous data, use from scratch
        	cv::Mat rv,tv;
        	auto solutions=IPPE::solvePnP_(object_points , imgPoints, camMatrix , distCoeffs);
        	double errorRatio=solutions[1].second/solutions[0].second;
        	if (errorRatio<minerrorRatio) return false;//is te error ratio big enough
        	cv::solvePnP(object_points,imgPoints, camMatrix, distCoeffs, rv, tv);

            rv.convertTo(rvec,CV_32F);
            tv.convertTo(tvec,CV_32F);
        	impl__aruco_getRTfromMatrix44(solutions[0].first,rvec,tvec);
     	}
    	else{
            cv::solvePnP(object_points, imgPoints, camMatrix, distCoeffs, rvec, tvec,true);
    	}
    return true;
}

bool my_aruco::coordinate_transform(Mat &R, Mat &t )
{

	Mat rotation_matrix = Mat( 3, 3, CV_32F );
	Mat rotation_matrix_inverse = Mat( 3, 3, CV_32F );
	Mat tvec_inverse = Mat( 3, 1, CV_32F);

	Rodrigues( rvec,  rotation_matrix);
	double invert_suc;
	invert_suc = invert( rotation_matrix,  rotation_matrix_inverse,DECOMP_SVD);	
	if(invert_suc==0) return false;

	tvec_inverse = -rotation_matrix_inverse * tvec;
	
	R = rotation_matrix_inverse;
	t = tvec_inverse;
	return true;
}

bool my_aruco::add_coordinate(Mat &image)
{
	if(ids.size() > 0 && !rvec.empty()) {
		aruco::drawDetectedMarkers(image, corners, ids);
		aruco::drawAxis(image, camMatrix, distCoeffs,  rvec,  tvec,markerLength * 0.5f);
		return true;
	}
	else
		return false;
}



