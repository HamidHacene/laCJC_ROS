#include "lanelib/imgUtils.h"
#include "lanelib/lane.hpp"
#include <vector>
#include <string>
#include <stdlib.h>  /*abs*/
//=============================
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xindex_view.hpp"
#include "xtensor-blas/xlinalg.hpp"
//=============================

using namespace cv;
#define BIRDEYE_VIEW 0
#define NORMAL_VIEW 1

lane::lane(const Mat s) : m_frameWidth(s.cols), m_frameHeight(s.rows), m_matSrc(s.clone())
{
	m_tpl = imread("/home/hamid/Documents/S4/Projet_ROS/laCJC_ROS/ROS_package/src/control/road_det/data/body.png", CV_LOAD_IMAGE_COLOR);
}

const vector<Point2f> lane::computeSrcROI()
{
	//Defining the ROI points
	int x0_ = 280, x1_ = 480, x2_ = 730, x3_ = 30;
	int y0_ = 230, y1_ = 230, y2_ = 460, y3_ = 460;
	vector<Point2f> srcPts = {Point2f(x0_, y0_),
							  Point2f(x1_, y1_),
							  Point2f(x2_, y2_),
							  Point2f(x3_, y3_)};
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
	switch (flag)
	{
	case BIRDEYE_VIEW:
		// Get Transformation Matrix
		m_transformationMatrix = getPerspectiveTransform(src, destPts);
		//Warping perspective
		warpPerspective(input, output, m_transformationMatrix, warpSize, INTER_LINEAR, BORDER_CONSTANT);
		break;
	case NORMAL_VIEW:
		m_transformationMatrix = getPerspectiveTransform(destPts, src);
		warpPerspective(input, output, m_transformationMatrix, warpSize, INTER_LINEAR);
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
	threshold(HSV_channels[i], HSV_channels[i], s_thresh_min, s_thresh_max, THRESH_OTSU);
	return HSV_channels[i];
}

xt::xarray<double> lane::polyfit2D(xt::xarray<double> &xValues, xt::xarray<double> &yValues)
{
	int n = xValues.size();
	xValues.reshape({n, 1});
	yValues.reshape({n, 1});
	xt::xarray<double> x2 = xt::pow(xValues, 2);
	xt::xarray<double> x0 = xt::ones<double>({n, 1});
	xt::xarray<double> A = xt::hstack(xt::xtuple(x2, xValues, x0));
	xt::xarray<double> At = xt::zeros<double>({3, n});
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			At(j, i) = A(i, j);
		}
	}
	A.reshape({n, 3});
	At.reshape({3, n});
	xt::xarray<double> N = xt::linalg::dot(At, A);
	auto N_ = xt::linalg::inv(N);
	auto coef = xt::linalg::dot(xt::linalg::dot(N_, At), yValues);
	return coef;
}

xt::xarray<double> lane::fullSearch(const Mat RoI, const xt::xarray<double> ploty, const string s0)
{
	//Set height of windows
	int nWindows = 13;
	int windowHeight = RoI.rows / nWindows;

	//Create empty lists to receive left lane pixel indices
	vector<int> center_lane_inds;
	vector<int> left_lane_inds;
	vector<int> right_lane_inds;

	vector<int> line_center_x;
	vector<int> line_center_y;
	//For visualization
	Mat RoIcol;
	cvtColor(RoI, RoIcol, CV_GRAY2RGB);

	//Loop
	for (int w = 0; w < nWindows; w++)
	{
		//Identify window boundaries in x and y (and right and left)
		int win_y_low = w * windowHeight;
		int win_y_high = (w + 1) * windowHeight;

		//For each window, we increment currLeftIndex until the sum of the row of index currLeftIndex is superior to the threshold 255 * windowHeight * .3
		//We decrement currRightIndex in the same way
		int col_sum = 0;
		int currLeftIndex = 0;
		while (col_sum < 255 * windowHeight * .3 and currLeftIndex < RoI.cols)
		{
			currLeftIndex++;
			col_sum = 0;
			for (int i = win_y_low; i < win_y_high; i++)
			{
				Point pt(currLeftIndex, i);
				col_sum += RoI.at<unsigned char>(pt);
			}
		}

		col_sum = 0;
		int currRightIndex = RoI.cols - 1;
		while (col_sum < 255 * windowHeight * .3 and currRightIndex >= 0)
		{
			currRightIndex--;
			col_sum = 0;
			for (int i = win_y_low; i < win_y_high; i++)
			{
				Point pt(currRightIndex, i);
				col_sum += RoI.at<unsigned char>(pt);
			}
		}

		left_lane_inds.push_back(currLeftIndex);
		right_lane_inds.push_back(currRightIndex);
		if (right_lane_inds[w] - left_lane_inds[w] > 0) //if a box was found
		{
			line_center_x.push_back((right_lane_inds[w] + left_lane_inds[w]) / 2);
			line_center_y.push_back(w*windowHeight);
			Point pt_low(left_lane_inds[w], w*windowHeight);
			Point pt_high(right_lane_inds[w], (w+1)*windowHeight);
			rectangle(RoIcol, pt_low, pt_high, Scalar(0, 255, 0), 2, 8, 0);
			if (s0 == "ROIL")
			{
				Point pt_low(left_lane_inds[w], m_frameHeight/2 + w*windowHeight);
				Point pt_high(right_lane_inds[w], m_frameHeight/2 + (w+1)*windowHeight);
				rectangle(m_BEV, pt_low, pt_high, Scalar(0, 255, 0), 2, 8, 0);
			}
			else
			{
				Point pt_low(m_frameWidth/2 + left_lane_inds[w], m_frameHeight/2 + w*windowHeight);
				Point pt_high(m_frameWidth/2 + right_lane_inds[w], m_frameHeight/2 + (w+1)*windowHeight);
				rectangle(m_BEV, pt_low, pt_high, Scalar(0, 255, 0), 2, 8, 0);
			}			
		}
	}
	std::vector<std::size_t> shape = { line_center_x.size() };
	if(line_center_x.size()<4)
	{
		if (s0 == "ROIL"){m_foundL=0;}
		else{m_foundR = 0;}
		return -1;
	}
	else
	{
		//Extract left line pixel positions
		xt::xarray<double> leftx = xt::adapt(line_center_x, shape);
		xt::xarray<double> lefty = xt::adapt(line_center_y, shape);
		//Compute the polynomial coefficient to fit with the line (2deg pol : ay² + by + c)
		xt::xarray<double> left_fit = polyfit2D(lefty, leftx);
		//Visualize the line
		auto left_fitx = left_fit(0, 0) * (xt::pow(ploty, 2)) + left_fit(1, 0) * ploty + left_fit(2, 0);
		int xb = left_fitx[left_fitx.size() - 1];
		if (s0 == "ROIL")
		{
			m_foundL = 1;
			m_bottom_l = xb;
		}
		else
		{
			m_foundR = 1;
			m_bottom_r = xb;
		}
		for (int j = 0; j < ploty.size(); j++)
		{
			Point2f zz(left_fitx(j), ploty(j));
			circle(RoIcol, zz, 1, CV_RGB(255, 0, 0));
			if (s0 == "ROIL")
			{
				Point2f zz(left_fitx(j), m_frameHeight/2 + ploty(j));
				circle(m_BEV, zz, 1, CV_RGB(255, 0, 0));
			}
			else
			{
				Point2f zz(m_frameWidth/2 + left_fitx(j), m_frameHeight/2 + ploty(j));
				circle(m_BEV, zz, 1, CV_RGB(255, 0, 0));
			}
		}
		imshow(s0, RoIcol);
		//imwrite("../data/fit.png", RoIcol);
		return left_fitx; //left_fitx = aY² + bY + c
	}
}

void lane::computeLaneCurvature(const xt::xarray<double> ploty, const xt::xarray<double> leftx, const xt::xarray<double> rightx)
{
	//Choose maximum y-value --> bottom of the image
	xt::xarray<double> y_eval = xt::amax(ploty);
	y_eval = 512;
	//Conversion in x & y from pixels -> meters
	double ym_per_pix = 7. / m_frameHeight;
	double xm_per_pix = LANEWIDTH / m_frameWidth;
	
	if(m_foundR==0 and m_foundL==0)
	{
		m_curveDir = 0;
		m_curveRad = 1500;
	}
	else if(m_foundR==0 and m_foundL==1)
	{
		//polyfit in world space
		xt::xarray<double> tmp1 = ym_per_pix * ploty;
		xt::xarray<double> tmp2 = xm_per_pix * leftx;
		xt::xarray<double> left_fit_cr = polyfit2D(tmp1, tmp2);
		//Compute radius of curvature in meters : R = ((1+(2Ay+B)²)^(3/2))/(|2A|)
		m_leftCurveRad = pow((1 + pow(2 * left_fit_cr(0) * y_eval * ym_per_pix + left_fit_cr(1), 2)), 1.5) / abs(2 * left_fit_cr(0));
		m_curveRad = m_leftCurveRad;
		//Find curve direction
		if ((leftx(0) - leftx(leftx.size() - 1)) > 50)
		{
			m_curveDir = 1; //right
		}
		else if ((leftx(leftx.size() - 1) - leftx(0)) > 50)
		{
			m_curveDir = -1; //left
		}
		else
		{
			m_curveDir = 0; //straight
		}		
	}
	else if(m_foundR==1 and m_foundL==0)
	{
		//polyfit in world space
		xt::xarray<double> tmp1 = ym_per_pix * ploty;
		xt::xarray<double> tmp3 = xm_per_pix * rightx;
		xt::xarray<double> right_fit_cr = polyfit2D(tmp1, tmp3);
		//Compute radius of curvature in meters : R = ((1+(2Ay+B)²)^(3/2))/(|2A|)
		m_rightCurveRad = pow((1 + pow(2 * right_fit_cr(0) * y_eval * ym_per_pix + right_fit_cr(1), 2)), 1.5) / abs(2 * right_fit_cr(0));
		m_curveRad = m_rightCurveRad;
		//Find curve direction
		if ((rightx(0) - rightx(rightx.size() - 1)) > 50)
		{
			m_curveDir = 1; //right
		}
		else if ((rightx(rightx.size() - 1) - rightx(0)) > 50)
		{
			m_curveDir = -1; //left
		}
		else
		{
			m_curveDir = 0; //straight
		}	}
	else if(m_foundR==1 and m_foundL==1)
	{
		//polyfit in world space
		xt::xarray<double> tmp1 = ym_per_pix * ploty;
		xt::xarray<double> tmp2 = xm_per_pix * leftx;
		xt::xarray<double> tmp3 = xm_per_pix * rightx;
		xt::xarray<double> left_fit_cr = polyfit2D(tmp1, tmp2);
		xt::xarray<double> right_fit_cr = polyfit2D(tmp1, tmp3);
		//Compute radius of curvature in meters : R = ((1+(2Ay+B)²)^(3/2))/(|2A|)
		m_leftCurveRad = pow((1 + pow(2 * left_fit_cr(0) * y_eval * ym_per_pix + left_fit_cr(1), 2)), 1.5) / abs(2 * left_fit_cr(0));
		m_rightCurveRad = pow((1 + pow(2 * right_fit_cr(0) * y_eval * ym_per_pix + right_fit_cr(1), 2)), 1.5) / abs(2 * right_fit_cr(0));
		m_curveRad = (m_leftCurveRad + m_rightCurveRad) / 2;
		//Find curve direction
		if ((leftx(0) - leftx(leftx.size() - 1)) > 50)
		{
			m_curveDir = 1; //right
		}
		else if ((leftx(leftx.size() - 1) - leftx(0)) > 50)
		{
			m_curveDir = -1; //left
		}
		else
		{
			m_curveDir = 0; //straight
		}
	}
}


Mat lane::thresholdRight()
{
	//detection of the right line
	/*Mat bin;
	const int max_value = 255;
	int Rmin = 137, Gmin = 163, Bmin = 157;
	int Rmax = 175, Gmax = 255, Bmax = 238;
	inRange(m_BEV, Scalar(Rmin, Gmin, Bmin), Scalar(Rmax, Gmax, Bmax), bin);
	return bin;*/

	Mat bin, hsv;
	int H_min = 0, S_min = 6, V_min = 160;
	int H_max = 180, S_max = 48, V_max = 255;

	Mat frame, frame_HSV, frame_threshold, res;
	//RGB to gray
	cvtColor(m_BEV, frame_HSV, COLOR_BGR2HSV);

	// Selection de la couleur
	inRange(frame_HSV, Scalar(H_min, S_min, V_min), Scalar(H_max, S_max, V_max), bin);

	int element_shape = MORPH_RECT;
	int an = 1;
	Mat element = getStructuringElement(element_shape, Size(an * 2 + 1, an * 2 + 1), Point(-1, -1));
	morphologyEx(bin, bin, MORPH_CLOSE, element);
	morphologyEx(bin, bin, MORPH_OPEN, element);

	return bin;
}

Mat lane::thresholdLeft()
{
	//detection of the left line
	/*Mat bin;
	const int max_value = 255;
	int Rmin = 137, Gmin = 163, Bmin = 157;
	int Rmax = 175, Gmax = 255, Bmax = 238;
	inRange(m_BEV, Scalar(Rmin, Gmin, Bmin), Scalar(Rmax, Gmax, Bmax), bin);
	return bin;*/

	Mat bin, hsv, tmp;
	int H_min = 21, S_min = 71, V_min = 143;
	int H_max = 180, S_max = 176, V_max = 255;

	Mat frame, frame_HSV, frame_threshold, res;
	//RGB to gray
	cvtColor(m_BEV, frame_HSV, COLOR_BGR2HSV);

	// Selection de la couleur
	inRange(frame_HSV, Scalar(H_min, S_min, V_min), Scalar(H_max, S_max, V_max), bin);

	int element_shape = MORPH_RECT;
	int an = 1;
	Mat element = getStructuringElement(element_shape, Size(an * 2 + 1, an * 2 + 1), Point(-1, -1));
	morphologyEx(bin, bin, MORPH_CLOSE, element);
	morphologyEx(bin, bin, MORPH_OPEN, element);
	return bin;
}

double lane::computeCarOffcenter(const xt::xarray<double> leftx, const double mid, const xt::xarray<double> rightx)
{
	double offset;
	if(m_foundR==0 and m_foundL==0)
	{
		offset = 1.3;
	}
	else if(m_foundR==0 and m_foundL==1)
	{
		double bottom_l = m_bottom_l;
		double a = mid - bottom_l;
		double width = 250;
		circle(m_BEV, Point(bottom_l, 500), 5, CV_RGB(0, 0, 0), -1);
		circle(m_BEV, Point(mid, 500), 5, CV_RGB(0, 0, 255), -1);
		offset = a / width * LANEWIDTH - LANEWIDTH / 2.0;
	}
	else if(m_foundR==1 and m_foundL==0)
	{
		double bottom_r = m_bottom_r + m_frameWidth/2;
		double b = bottom_r - mid;
		double width = 250;
		circle(m_BEV, Point(bottom_r, 500), 5, CV_RGB(0, 0, 0), -1);
		circle(m_BEV, Point(mid, 500), 5, CV_RGB(0, 0, 255), -1);
		offset = LANEWIDTH / 2.0 - b / width * LANEWIDTH;
	}
	else if(m_foundR==1 and m_foundL==1)
	{
		double bottom_l = m_bottom_l;
		double bottom_r = m_bottom_r + m_frameWidth/2;
		double a = mid - bottom_l;
		double b = bottom_r - mid;
		double width = bottom_r - bottom_l;

		circle(m_BEV, Point(bottom_l, 500), 5, CV_RGB(0, 0, 0), -1);
		circle(m_BEV, Point(bottom_r, 500), 5, CV_RGB(0, 0, 0), -1);
		circle(m_BEV, Point(mid, 500), 5, CV_RGB(0, 0, 255), -1);
		if (a >= b)
		{
			offset = a / width * LANEWIDTH - LANEWIDTH / 2.0;
		}
		else
		{
			offset = LANEWIDTH / 2.0 - b / width * LANEWIDTH;
		}
	}
	return offset;
}

double lane::computeMid(const Mat M, const Mat templ)
{
	Mat img_display, result;
	result.create(M.cols - templ.cols + 1, M.rows - templ.rows + 1, CV_32FC1);
	//match the template
	matchTemplate(M, templ, result, 3);
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	//find the loction of the template in the image
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
	double x0 = maxLoc.x + templ.cols / 2, y0 = maxLoc.y + templ.rows;
	//Now, we need to convert the (x,y) from the source image to (x',y') in BirdEye View
	vector<Point2f> srcPts = computeSrcROI();
	Mat ss = transformingView(m_BEV, 1, srcPts);
	xt::xarray<double> mInv = xt::zeros<double>({3, 3});
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			mInv(i, j) = m_transformationMatrix.at<double>(i, j);
		}
	}
	//We need to inverse the matrix to have : SRC --> BirdEyeView
	mInv = xt::linalg::inv(mInv);
	//We apply the transformation matrix to the point
	int xn1 = (mInv(0, 0) * x0 + mInv(0, 1) * y0 + mInv(0, 2)) / (mInv(2, 0) * x0 + mInv(2, 1) * y0 + mInv(2, 2));
	//We only need the x position
	return xn1;
}

void lane::processFrame()
{
	//Build Bird Eye View
	BirdEyeView();
	line(m_BEV, Point(0,m_frameHeight/2), Point(m_frameWidth,m_frameHeight/2), CV_RGB(255, 255, 0));
	//Apply a thresh to extract Left line
	Mat S = thresholdLeft();
	int x0 = 0, y0 = m_frameHeight/2, w0 = m_frameWidth/2, h0 = m_frameHeight/2;
	Rect Rec1(x0, y0, w0, h0);
	Mat ROIL;
	S(Rec1).copyTo(ROIL);

	//Apply a thresh to extract Right line
	Mat SR = thresholdRight();
	x0 = m_frameWidth / 2, y0 = m_frameHeight / 2, w0 = m_frameWidth / 2, h0 = m_frameHeight / 2;
	Rect Rec2(x0, y0, w0, h0);
	Mat ROIR;
	SR(Rec2).copyTo(ROIR);

	//Full window Search
	m_ploty = xt::linspace<double>(0, ROIR.rows - 1, ROIR.rows);
	xt::xarray<double> left_fitx = fullSearch(ROIL, m_ploty, "ROIL");
	xt::xarray<double> right_fitx = fullSearch(ROIR, m_ploty, "ROIR");
	//Compute lane curvature
	computeLaneCurvature(m_ploty, left_fitx, right_fitx);
	//Compute car offcenter
	double midl = computeMid(m_matSrc, m_tpl);
	m_offCenter = computeCarOffcenter(left_fitx, midl, right_fitx);
}
//=================================================
/*Visualization functions*/
//=================================================
void lane::showTxtData()
{
	int fontFace = FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;

	string rs = "Road Status";
	int baseline = 0;
	Size textSize = getTextSize(rs, fontFace, fontScale, thickness, &baseline);
	putText(m_matSrc, rs, Point(0, textSize.height), fontFace, fontScale, Scalar::all(255), thickness, 8);

	fontScale = 0.8;
	thickness = 2;
	rs = "   Lane info: ";
	if (m_curveDir == 0)
		rs += "Straight";
	else if (m_curveDir == -1)
		rs += "Left curve";
	else
		rs += "Right curve";

	baseline = 0;
	textSize += getTextSize(rs, fontFace, fontScale, thickness, &baseline);
	putText(m_matSrc, rs, Point(0, textSize.height + 8), fontFace, fontScale, Scalar::all(255), thickness, 8);

	rs = "   Curvature: " + std::to_string(m_curveRad(0)) + " m";
	textSize += getTextSize(rs, fontFace, fontScale, thickness, &baseline);
	putText(m_matSrc, rs, Point(0, textSize.height + 16), fontFace, fontScale, Scalar::all(255), thickness, 8);

	if (m_offCenter < 0)
		rs = "   Off center: Left " + std::to_string(abs(m_offCenter)) + " m";
	else if (m_offCenter > 0)
		rs = "   Off center: Right " + std::to_string(abs(m_offCenter)) + " m";
	else
		rs = "   Off center: Straight 0 m";
	textSize += getTextSize(rs, fontFace, fontScale, thickness, &baseline);
	putText(m_matSrc, rs, Point(0, textSize.height + 24), fontFace, fontScale, Scalar::all(255), thickness, 8);
}

void lane::buildVisu(const string s)
{
	showTxtData();
	Mat visual(Size(m_matSrc.cols * 2, m_matSrc.rows), m_matSrc.type(), Scalar::all(0));
	Mat matRoi = visual(Rect(0, 0, m_matSrc.cols, m_matSrc.rows));
	m_matSrc.copyTo(matRoi);

	matRoi = visual(Rect(m_matSrc.cols, 0, m_matSrc.cols, m_matSrc.rows));
	m_BEV.copyTo(matRoi);

	resize(visual, visual, Size(), 0.75, 0.75);
	imshow(s, visual);
}