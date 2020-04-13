#include "imgUtils.h"
#include "lane.hpp"
#include <vector>
//=============================
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xindex_view.hpp"
#include "xtensor-blas/xlinalg.hpp"
//=============================

using namespace cv;
#define BIRDEYE_VIEW 0
#define NORMAL_VIEW  1

lane::lane(const Mat s): m_frameWidth(s.cols), m_frameHeight(s.rows), m_matSrc(s.clone())
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

void lane::BirdEyeView()
{
	vector<Point2f> srcPts = computeSrcROI();
	m_BEV = transformingView(m_matSrc, 0, srcPts);
}

Mat lane::thresholdColChannel(int i, int s_thresh_min, int s_thresh_max)
{
	//detection of the left line
	cvtColor(m_BEV, m_HSV, COLOR_BGR2HSV);
	vector<Mat> HSV_channels(3);
	split(m_HSV, HSV_channels);
	threshold(HSV_channels[i], HSV_channels[i], s_thresh_min , s_thresh_max , THRESH_OTSU);
	//threshold(HSV_channels[i], m_HSV, s_thresh_min , s_thresh_max , THRESH_OTSU);
	//merge(HSV_channels, m_HSV);
	return HSV_channels[i];
}

xt::xarray<double> lane::polyfit2D(xt::xarray<int> &xValues, xt::xarray<int> &yValues)
{
	int n = xValues.size();
	xValues.reshape({n,1});
	yValues.reshape({n,1});
	xt::xarray<double> x2 = xt::pow(xValues, 2);
	xt::xarray<double> x0 = xt::ones<double>({n,1});
	xt::xarray<double> A = xt::hstack(xt::xtuple(x2, xValues, x0));
	xt::xarray<int> At = xt::zeros<double>({3,n});
	for(int i=0; i<n; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			At(j,i) = A(i,j);
		}
	}
	A.reshape({n,3});
	At.reshape({3,n});
	xt::xarray<double> N = xt::linalg::dot(At, A);
	auto N_ = xt::linalg::inv(N);
	auto coef = xt::linalg::dot(xt::linalg::dot(N_, At), yValues);
	return coef;
}

xt::xarray<double> lane::fullSearch(const Mat RoI, const xt::xarray<double> ploty)
{
//Compute the starting base of the first window
/*
	Some improvement is needed here : since we have a two line lane, we need to
	compute the barycenter of the maximums below.
*/
	Mat histo;
	reduce(RoI, histo, 0, CV_REDUCE_SUM, CV_32S);
	Point min_loc, max_loc;
	double min, max;
	minMaxLoc(histo, &min, &max, &min_loc, &max_loc);
//Set height of windows
	int nWindows = 13;
	int windowHeight = RoI.rows/nWindows;
//find x and y position of non-zero pixels
	Mat nonZero;
	findNonZero(RoI, nonZero);
	auto nonZeroX = xt::xarray<int>::from_shape({nonZero.total()});
	auto nonZeroY = xt::xarray<int>::from_shape({nonZero.total()});
	for (int i = 0; i < nonZero.total(); i++ ) 
	{
		nonZeroX(i) = nonZero.at<Point>(i).x;
		nonZeroY(i) = nonZero.at<Point>(i).y;
	}
//current positions to be updated for each window
	int leftx_current = max_loc.x;
//Set the width of the window +/- margin
	int margin = 30;
//Set minimum number of pixels to be found to recenter window
	int minPix = 50;
/*the two values given here are arbitrary (they lead to correct results)*/
//Create empty lists to receive left lane pixel indices
	vector<int> left_lane_inds;
//For visualization
	Mat RoIcol;
	cvtColor(RoI, RoIcol, CV_GRAY2RGB);
	//Loop
	for(int w=0; w<nWindows; w++)
	{
	//Identify window boundaries in x and y (and right and left)
		int win_y_low = RoI.rows - (w+1)*windowHeight;
		int win_y_high = RoI.rows - w*windowHeight;
		int win_xleft_low = leftx_current - margin;
		int win_xleft_high = leftx_current + margin;
		Point pt_low(win_xleft_low, win_y_low);
		Point pt_high(win_xleft_high, win_y_high);
		circle(RoIcol, pt_low, 5, CV_RGB(255,0,255));
		circle(RoIcol, pt_high, 5, CV_RGB(255,255,0));	
	//Visualization
		rectangle(RoIcol, pt_low, pt_high, Scalar(0,255,0), 2, 8, 0);
	//Identify the nonzero pixels within the window
		vector<int> good_left_inds;
		for(int i = 0; i < nonZero.total(); i++)
		{
			if((nonZeroY(i)>=win_y_low)&(nonZeroY(i)<win_y_high)&(nonZeroX(i)>=win_xleft_low)&(nonZeroX(i)<win_xleft_high))
			{
				good_left_inds.push_back(i);
			}
		}
	//Append the indecis to the list
		left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
	//If we found > minPix pixels, recenter the next window on the mean position
		if (good_left_inds.size()>minPix)
		{
			auto b = xt::index_view(nonZeroX, good_left_inds);
			leftx_current = xt::mean(b)();
		}
		good_left_inds.clear();
	}
//Extract left line pixel positions
	xt::xarray<int> leftx = xt::index_view(nonZeroX, left_lane_inds);
	xt::xarray<int> lefty = xt::index_view(nonZeroY, left_lane_inds);
//Compute the polynomial coefficient to fit with the line (2deg pol : ay^² + by + c)
	xt::xarray<double> left_fit = polyfit2D(lefty, leftx);
//Visualize the line
	int n = lefty.size();	
	lefty.reshape({n});
	leftx.reshape({n});
	int yb = RoI.rows;
	int xb = max_loc.x;
	auto left_fitx = left_fit(0)*(xt::pow(ploty, 2)) + left_fit(1)*ploty + left_fit(2);
	for(int j=0; j<n; j++)
	{
		Point2f zz(xb+left_fitx(j), yb-ploty(j));
		circle(RoIcol, zz, 1, CV_RGB(255,0,0));
	}
	imshow("RoI", RoIcol);
	return left_fitx; //X = aY² + bY + c
}

void lane::processFrame()
{
//Build Bird Eye View
	BirdEyeView();
//Apply a thresh on S channel
	Mat S = thresholdColChannel();
//Select the appropriate ROI (here we oonly select the left one)
//To use the right line, a more accurate filter is needed to extract the line !
//Need improvements ! 
	int x0 = 0, y0 = m_frameHeight/2, w0 = m_frameWidth/2 , h0 = m_frameHeight/2;
	Rect Rec(x0, y0, w0, h0);
	Mat ROI; 
	S(Rec).copyTo(ROI);
//Full window Search
	auto ploty = xt::linspace<double>(0, ROI.rows-1, ROI.rows);
	xt::xarray<double> left_fitx = fullSearch(ROI, ploty);
/*
Next steps : 
	* stocker mean(left_fitx) (centre)
	* compute curvature;
	* compute car's off-center distance;
	* build visualisation;
*/
}
