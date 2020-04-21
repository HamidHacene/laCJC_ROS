#include <ros/ros.h>
#include "std_msgs/Float64.h"
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
static const std::string res = "Result";
 
class ImageConverter
{
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	const ros::Publisher *carOffCenter;
	const ros::Publisher *carCenter;

public:
	ImageConverter(const ros::NodeHandle *nh_, const ros::Publisher *pub, const ros::Publisher *pub2) : it_(*nh_)
	{
		image_sub_ = it_.subscribe("/image", 1, &ImageConverter::process, this);
		cv::namedWindow(res);
		carOffCenter = pub;
		carCenter = pub2;
 	}
 
	~ImageConverter()
	{
		cv::destroyAllWindows();
	}

	void process(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
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
		//std::cout << "Curve radius = " << L.m_curveRad << std::endl;
		//std::cout << "Curve direction = " << L.m_curveDir << std::endl;
		//std::cout << "OffCenter = " << L.m_offCenter << std::endl;

		std_msgs::Float64 message;
		message.data = L.m_offCenter;
		carOffCenter->publish(message);

		std_msgs::Float64 message2;
		message2.data = 0.0;
		carCenter->publish(message2);
		L.buildVisu(res);
		//imshow(imSrc, L.m_matSrc);
		//imshow(birdView, L.m_BEV);
		cv::waitKey(1);
 		//----------------------------------------------------------------------------------------------------
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh_;
	ros::Publisher carOffCenter = nh_.advertise<std_msgs::Float64>("/state", 100);
	ros::Publisher carCenter = nh_.advertise<std_msgs::Float64>("/setpoint", 100);

	ImageConverter ic(&nh_, &carOffCenter, &carCenter);

	ros::spin();
	return 0;
}
