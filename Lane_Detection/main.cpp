#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
//#include <math.h>
#include <vector>

#include "imgUtils.h"
#include "lane.hpp"
//=============================
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xindex_view.hpp"
#include "xtensor-blas/xlinalg.hpp"
//=============================


using namespace cv;
using namespace std;

static void help(char **argv) 
{
	cout << endl
		 << "This program demonstrated the use of the discrete Fourier transform (DFT). " << endl
		 << "The dft of an image is taken and it's power spectrum is displayed." << endl << endl
		 << "Usage:" << endl
		 << argv[0] << " [image_name -- default lena.jpg]" << endl << endl;
}

xt::xarray<double> polyfit2D(xt::xarray<int> &xValues, xt::xarray<int> &yValues)
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
	//cout << A << endl;
	//cout << At << endl;

	xt::xarray<double> N = xt::linalg::dot(At, A);
	auto N_ = xt::linalg::inv(N);
	auto X = xt::linalg::dot(xt::linalg::dot(N_, At), yValues);
	return X;
}


//=============================================================================================
int main(int argc, char **argv)
{
	//xt::xarray<int> x = {1,2,3,4,5,6};
	//xt::xarray<int> y = {1,4,9,16,25,36};
	//cout << polyfit2D(y, x) << endl;
	Mat I = imread("../data/virageD.png", CV_LOAD_IMAGE_COLOR);
	lane L(I);

	L.BirdEyeView();
	Mat S = L.thresholdColChannel();

//ROI + leftx_base
	//Choisir la RoI
	int x0 = 0, y0 = L.m_frameHeight/2, w0 = L.m_frameWidth/2 , h0 = L.m_frameHeight/2;
	Rect Rec(x0, y0, w0, h0);
	Mat RoI; 
	S(Rec).copyTo(RoI);
	//Calculer la base de depart de la window
	Mat histo;
	reduce(RoI, histo, 0, CV_REDUCE_SUM, CV_32S);
	//int midpoint = histo.cols/2;
	Point min_loc, max_loc;
	double min, max;
	minMaxLoc(histo, &min, &max, &min_loc, &max_loc);
//Suite du traitement : 
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
	//cout << "nzX = " << nonZeroX << endl;
	//cout << "nzY = " << nonZeroY << endl;
	//current positions to be updated for each window
	int leftx_current = max_loc.x;
	//Set the width of the window +/- margin
	int margin = 60;
	//Set minimum number of pixels to be found to recenter window
	int minPix = 30;
	//Create empty lists to receive left lane pixel indices
	vector<int> left_lane_inds;
	//Pour l'affichage en couleur
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

	xt::xarray<double> left_fit = polyfit2D(lefty, leftx);
	cout << "left_fit = " << left_fit << endl;
//tracer les somme et barycentre des deux sommes !
	cout << RoI(2) << endl;
	imshow("RoI", RoIcol);
	//imgUtils::computeHistogramme(S,2);

	//imshow("Im_source", L.m_matSrc);
	imshow("Bird Eye View", L.m_BEV);
	//imshow("Thresh S", S);

	waitKey(0);
	destroyAllWindows();	
	return EXIT_SUCCESS;
}

