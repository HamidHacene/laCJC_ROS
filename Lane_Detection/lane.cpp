#include "imgUtils.h"
#include "lane.hpp"
#include <vector>

using namespace cv;
#define BIRDEYE_VIEW 0
#define NORMAL_VIEW  1

lane::lane(const Mat s): m_frameWidth(s.rows), m_frameHeight(s.cols), m_matSrc(s.clone())
{
	//Init
}

const vector<Point2f> lane::computeSrcROI()
{
	//Defining the ROI points
	int x0_=280,x1_=480,x2_=730,x3_=30;
	int y0_=230,y1_=230,y2_=460,y3_=460;
	vector<Point2f> srcPts = {Point2f(x0_,y0_), 
						  Point2f(x1_,y1_),
						  Point2f(x2_,y2_),
						  Point2f(x3_,y3_)};	
	return srcPts;
}

const Mat lane::transformingView(const Mat input, const int flag, const vector<Point2f> src)
{
	//Change perspective from driver view to bird eye view	
	//Param
	//Input : Mat input image frame
	//		  int flag = 0 for "BIRDEYE_VIEW" or 1 for "NORMAL_VIEW"
	vector<Point2f> destPts = {Point2f(200, 0), 
							   Point2f(500, 0),
							   Point2f(500, 511), 
							   Point2f(200, 511)};
	Size warpSize(input.size());
	Mat output(warpSize, input.type());
	Mat transformationMatrix;
	switch(flag)
	{
		case BIRDEYE_VIEW:
			// Get Transformation Matrix
			transformationMatrix = getPerspectiveTransform(src, destPts);
			//Warping perspective
			warpPerspective(input, output, transformationMatrix, warpSize, INTER_LINEAR, BORDER_CONSTANT);
			break;
		case NORMAL_VIEW:
			transformationMatrix = getPerspectiveTransform(destPts, src);
			warpPerspective(input, output, transformationMatrix, warpSize,INTER_LINEAR);
			break;
		default:
			cerr << "ERROR: FLAG ERROR\n";
			break;
	}
	return output;
}

const Mat lane::BirdEyeView()
{
	vector<Point2f> srcPts = computeSrcROI();
	m_BEV = transformingView(m_matSrc, 0, srcPts);
	return m_BEV;
}