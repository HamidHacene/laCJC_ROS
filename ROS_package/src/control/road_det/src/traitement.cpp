//https://www.youtube.com/watch?v=yvfI4p6Wyvk
//https://www.youtube.com/watch?v=u2mmfdRicSQ

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>

using namespace std;
using namespace cv;

//==============================
//===Global Vars================
//==============================
static const string OPENCV_WINDOW = "test window";
static const string OPENCV_WINDOW1 = "birdView window";

#define BIRDEYE_VIEW 0
#define NORMAL_VIEW  1
//interest area
const vector<Point2f> srcPts = {Point2f(320,190), 
								Point2f(445,190),
								Point2f(730,410),
								Point2f(85,410)};

/*const vector<Point2f> destPts = {Point2f(110,540), 
								 Point2f(110, 0),
        						 Point2f(800,0), Point2f(800,540)
    							};*/
//==============================
//==============================

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
		image_sub_ = it_.subscribe("/image", 1, &ImageConverter::detection, this);
		//image_pub_ = it_.advertise("/image_converter/output", 1);
		namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		destroyWindow(OPENCV_WINDOW);
	}

	void detection(const sensor_msgs::ImageConstPtr& msg)
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
	//ToDo Here : use cv_ptr->image
	//---------------------------------------------------------------------------------------------------
		//ROS_INFO("Shape : width = [%d] // height = [%d]", cv_ptr->image.cols, cv_ptr->image.rows);
		//cout << srcPts[0].x << "      " << srcPts[0].y << endl;
		Mat birdView = transformingView(cv_ptr->image, 0);

		circle(cv_ptr->image, srcPts[0], 10, CV_RGB(255,0,0));
		circle(cv_ptr->image, srcPts[1], 10, CV_RGB(0,255,0));
		circle(cv_ptr->image, srcPts[2], 10, CV_RGB(0,0,255));	
		circle(cv_ptr->image, srcPts[3], 10, CV_RGB(255,255,255));
		imshow(OPENCV_WINDOW, cv_ptr->image);
		imshow(OPENCV_WINDOW1, birdView);		
		waitKey(0);
		destroyAllWindows();
		// Output modified video stream
		//image_pub_.publish(cv_ptr->toImageMsg());
	//----------------------------------------------------------------------------------------------------
	}

	void laneDetection(Mat img)
	{
		Mat gray;
		cvtColor(img, gray, CV_BGR2GRAY);
		imshow(OPENCV_WINDOW1, gray);
	}

	vector<Point2f> compute4Pts_Transform(vector<Point2f> src)
	{	
		Point2f tl = src[0], tr = src[1], bl = src[2], br = src[3];
	//compute the width of the new image	
		float widthA = sqrt( pow((br.x - bl.x),2) + pow((br.y - bl.y),2));
		float widthB = sqrt( pow((tr.x - tl.x),2) + pow((tr.y - tl.y),2));
		long int maxWidth = lroundf(fmaxf(widthA, widthB));
		//cout << maxWidth << endl;
	//compute the height of the new image
		float heightA = sqrt( pow((tr.x - br.x),2) + pow((tr.y - br.y),2));
		float heightB = sqrt( pow((tl.x - bl.x),2) + pow((tl.y - bl.y),2));
		long int maxHeight = lroundf(fmaxf(heightA , heightB));
	//construct the destPts
		vector<Point2f> dest = {Point2f(0, 0), 
								   Point2f(maxWidth-1, 0),
								   Point2f(maxWidth-1, maxHeight-1), 
								   Point2f(0, maxHeight-1)};
		cout << dest << endl;
		return dest;
	}

	Mat transformingView(Mat input, const int flag)
	{
		//Change perspective from driver view to bird eye view	
		//Param
		//Input : Mat input image frame
		//		  int flag = 0 for "BIRDEYE_VIEW" or 1 for "NORMAL_VIEW"
		
		//vector<Point2f> destPts = compute4Pts_Transform(srcPts);
		const vector<Point2f> destPts = {Point2f(0,0), 
								Point2f(640,0),
								Point2f(640,480),
								Point2f(0,480)};
		//Size warpSize(destPts[3].y + 1, destPts[1].x + 1);
		Size warpSize(480, 640);
		//Size warpSize(input.size());
		Mat output(warpSize, input.type());
		Mat transformationMatrix;

		switch(flag)
		{
			case BIRDEYE_VIEW:
				// Get Transformation Matrix
				transformationMatrix = getPerspectiveTransform(srcPts, destPts);
				//Warping perspective
				warpPerspective(input, output, transformationMatrix, warpSize, INTER_LINEAR, BORDER_CONSTANT);
				break;
			case NORMAL_VIEW:
				transformationMatrix = getPerspectiveTransform(destPts, srcPts);
				warpPerspective(input, output, transformationMatrix, warpSize,INTER_LINEAR);
				break;
			default:
				cerr << "ERROR: FLAG ERROR\n";
				break;
		}
		return output;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}





// Draw an example circle on the video stream
/*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
{
	circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));
}*/