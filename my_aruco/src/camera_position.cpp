//my_aruco
#include "my_aruco.h"
//ros
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

//other
#include <math.h>
#include "match.h"

#include <iostream>
using namespace std;

bool got_image(false);
Mat  image;
tf::StampedTransform transform_old;

void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
 	got_image = true;
	ROS_INFO("image_ready");
	cv_bridge::CvImagePtr input_bridge;
	try {
		input_bridge = cv_bridge::toCvCopy(image_msg, 	sensor_msgs::image_encodings::	MONO8);
		image = input_bridge->image;
	}
	catch (cv_bridge::Exception& ex){
	        ROS_ERROR("[draw_frames] Failed to convert image");

        return;
	}
}

void SubTopics_Init(void)
{
	static ros::NodeHandle nstru;
	static image_transport::ImageTransport it(nstru);
	static image_transport::Subscriber sub_image = it.subscribe("/image", 1, imageCallback);
}

void rosTopic_publish(geometry_msgs::PoseStamped position_msg,geometry_msgs::PoseStamped transform_msg)
{
	static ros::NodeHandle nstru;
	static ros::Publisher position_pub = nstru.advertise<geometry_msgs::PoseStamped>("/my_aruco/pose_out", 1);
	static ros::Publisher tf_pub = nstru.advertise<geometry_msgs::PoseStamped>("/my_aruco/odomold_pose_out", 1);
    position_pub.publish(position_msg);
	tf_pub.publish(transform_msg);

}

void TMainRunning()
{
    static my_aruco myaruco;
	static match mmatch;

	int num = myaruco.detect(image);

	Mat R,t;
	Mat cMo(4,4,CV_32F);
	Mat position;
	float direction;

	if(num!=0)
	{
//        ROS_INFO("ARUCO_FOUND");
        bool estimated(false);

        estimated = myaruco.estimatePose( );
        if(estimated == false) return;

        Mat image_draw;
        image.copyTo(image_draw);
        myaruco.add_coordinate(image_draw);
        imshow("image_draw",image_draw);
        myaruco.coordinate_transform(R,t);
        cMo.at<float>(0,0) = R.at<float>(0,0);
        cMo.at<float>(0,1) = R.at<float>(0,1);
        cMo.at<float>(0,2) = R.at<float>(0,2);
        cMo.at<float>(0,3) = t.at<float>(0);

        cMo.at<float>(1,0) = R.at<float>(1,0);
        cMo.at<float>(1,1) = R.at<float>(1,1);
        cMo.at<float>(1,2) = R.at<float>(1,2);
        cMo.at<float>(1,3) = t.at<float>(1);

        cMo.at<float>(2,0) = R.at<float>(2,0);
        cMo.at<float>(2,1) = R.at<float>(2,1);
        cMo.at<float>(2,2) = R.at<float>(2,2);
        cMo.at<float>(2,3) = t.at<float>(2);

        cMo.at<float>(3,0) = 0;
        cMo.at<float>(3,1) = 0;
        cMo.at<float>(3,2) = 0;
        cMo.at<float>(3,3) = 1;

        position = mmatch.position_to_base_coordinate(cMo);
        direction = mmatch.direction_to_base_coordinate(cMo);   //roll of camare,that is the yaw angle of robot
	}
    geometry_msgs::PoseStamped position_msg;
    geometry_msgs::PoseStamped transform_msg;

    if(position.size().height!=0)
    {
        position_msg.pose.position.x = position.at<float>(0);
        position_msg.pose.position.y = position.at<float>(1);
        position_msg.pose.position.z = direction;
        position_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(direction,0,0);
    }
    else
    {
        position_msg.pose.position.x = 0;
        position_msg.pose.position.y = 0;
        position_msg.pose.position.z = 0;
        position_msg.pose.orientation.x = 0;
        position_msg.pose.orientation.y = 0;
        position_msg.pose.orientation.z = 0;
        position_msg.pose.orientation.w = 1;
    }

    transform_msg.pose.position.x = transform_old.getOrigin().x();
    transform_msg.pose.position.y = transform_old.getOrigin().y();
    transform_msg.pose.position.z = transform_old.getOrigin().z();
    transform_msg.pose.orientation.w = transform_old.getRotation().w();
    transform_msg.pose.orientation.z = transform_old.getRotation().z();
    transform_msg.pose.orientation.y = transform_old.getRotation().y();
    transform_msg.pose.orientation.x = transform_old.getRotation().x();

    position_msg.header.stamp = ros::Time::now();
    transform_msg.header.stamp = position_msg.header.stamp;
    position_msg.header.frame_id = "map_r";
    transform_msg.header.frame_id = "map_r";

    rosTopic_publish(position_msg,transform_msg);

	waitKey(1);
}

void waitforimage()
{
    while (ros::ok()){
        ROS_INFO("waite_for_image");
		if (got_image) return;
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "myaruco");
	ros::NodeHandle nstru;
	ros::Rate loop_rate(10);

    tf::TransformListener listener;

	SubTopics_Init();
	waitforimage();
	while(ros::ok())
	{
        try{
          listener.lookupTransform("/map_r", "/base_link_r", ros::Time(0), transform_old);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

		TMainRunning();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
