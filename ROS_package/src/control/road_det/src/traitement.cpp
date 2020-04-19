#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include "lanelib/lane.hpp"

using namespace std;
using namespace cv;

//==============================
//===Global Vars================
//==============================
static const string OPENCV_WINDOW = "test window";
class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter(): it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/image", 1, &ImageConverter::process, this);
		//image_pub_ = it_.advertise("/image_converter/output", 1);
		namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		destroyWindow(OPENCV_WINDOW);
	}

	void process(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	//Code Here : use cv_ptr->image
	//---------------------------------------------------------------------------------------------------
		ROS_INFO("Shape : width = [%d] // height = [%d]", cv_ptr->image.cols, cv_ptr->image.rows);
		imshow(OPENCV_WINDOW, cv_ptr->image);
	//----------------------------------------------------------------------------------------------------
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}