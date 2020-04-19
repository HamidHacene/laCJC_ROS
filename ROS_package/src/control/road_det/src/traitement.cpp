#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include "lanelib/lane.hpp"

//=============================
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xindex_view.hpp"
#include "xtensor-blas/xlinalg.hpp"
//=============================

//==============================
//===Global Vars================
//==============================
static const std::string imSrc = "Im_source";
static const std::string birdView = "Bird_Eye_View";
static const std::string roiL = "ROIL";
static const std::string roiR = "ROIR";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

public:
	ImageConverter(): it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/image", 1, &ImageConverter::process, this);
		//image_pub_ = it_.advertise("/image_converter/output", 1);
		cv::namedWindow(imSrc);
	}

	~ImageConverter()
	{
		cv::destroyAllWindows();
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
		//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::Mat B = cv_ptr->image.clone(); 
		lane L(B);
		L.processFrame();
		std::cout << "Curve radius = " << L.m_curveRad << std::endl;
		std::cout << "Curve direction = " << L.m_curveDir << std::endl;
		std::cout << "OffCenter = " << L.m_offCenter << std::endl;
		imshow(imSrc, L.m_matSrc);
		imshow(birdView, L.m_BEV);		
		cv::waitKey(0);
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