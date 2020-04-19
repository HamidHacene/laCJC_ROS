#ifndef LANE_HPP
#define LANE_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <string>

//=============================
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xindex_view.hpp"
#include "xtensor-blas/xlinalg.hpp"
//=============================

using namespace cv;
using namespace std;


class lane
{
public:
	//Méthodes de classe
	lane(const Mat s);
	const vector<Point2f> computeSrcROI();
	const Mat transformingView(const Mat input, const int flag, const vector<Point2f> src);
	void BirdEyeView();
	Mat thresholdColChannel(int i = 1/*channel*/, const int s_thresh_min = 120, const int s_thresh_max = 255);
	xt::xarray<double> polyfit2D(xt::xarray<double> &xValues, xt::xarray<double> &yValues);
	xt::xarray<double> fullSearch(const Mat RoI, const xt::xarray<double> ploty, const string s0);
	void computeLaneCurvature(const xt::xarray<double> ploty, const xt::xarray<double> leftx, const xt::xarray<double> rightx);
	Mat thresholdRight();
	double computeCarOffcenter(const xt::xarray<double> leftx, const double mid, const xt::xarray<double> rightx);
	double computeMid(const Mat M, const Mat templ);
	void processFrame();

	//Variables d'instance
	int m_frameWidth = 0;
	int m_frameHeight = 0;
	int m_bottom_l, m_bottom_r;
	double LANEWIDTH = 3.75;
	Mat m_matSrc, m_BEV, m_HSV;
	Mat m_transformationMatrix, m_tpl;
	xt::xarray<double> m_ploty;
	xt::xarray<double> m_leftCurveRad, m_rightCurveRad, m_curveRad;
	int m_curveDir;
	double m_offCenter, m_mid;
};
#endif