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

using namespace cv;
using namespace std;


class lane
{
public:
	//Méthodes de classe
	lane(const Mat s);
	const Mat BirdEyeView();
	const Mat transformingView(const Mat input, const int flag, const vector<Point2f> src);
	const vector<Point2f> computeSrcROI();


	//Variables d'instance
	int m_frameWidth = 0;
	int m_frameHeight = 0;
	Mat m_matSrc, m_BEV;
};
#endif