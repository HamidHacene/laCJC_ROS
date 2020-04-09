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

//=============================================================================================
int main(int argc, char **argv)
{
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
	vector<int> nonZeroX, nonZeroY;
	for (int i = 0; i < nonZero.total(); i++ ) 
	{
		nonZeroX.push_back(nonZero.at<Point>(i).x);
		nonZeroY.push_back(nonZero.at<Point>(i).y);
	}
	//current positions to be updated for each window
	int leftx_current = max_loc.x;
	//Set the width of the window +/- margin
	int margin = 40;
	//Set minimum number of pixels to be found to recenter window
	int minPix = 20;
	//Create empty lists to receive left lane pixel indices
	vector<int> left_lane_inds;
	//Pour l'affichage en couleur
	Mat RoIcol;
	cvtColor(RoI, RoIcol, CV_GRAY2RGB);
	//Loop
	for(int w=0; w<nWindows; w++)
	{
		//Identify window boundaries in x and y (and right and left)
		Point pt_low(leftx_current - margin, RoI.rows - (w+1)*windowHeight);
		Point pt_high(leftx_current + margin, RoI.rows - w*windowHeight);
		rectangle(RoIcol, pt_low, pt_high, Scalar(0,255,0), 2, 8, 0);
	}

	

	
	imshow("RoI", RoIcol);
	//imgUtils::computeHistogramme(S,2);

	//imshow("Im_source", L.m_matSrc);
	imshow("Bird Eye View", L.m_BEV);
	//imshow("Thresh S", S);

	waitKey(0);
	destroyAllWindows();	
	return EXIT_SUCCESS;
}

