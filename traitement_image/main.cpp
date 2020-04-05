#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "imgUtils.h"
#include <iostream>
#include <string>
#include <math.h>

using namespace cv;
using namespace std;

static void help(char **argv);
//interest area
const vector<Point2f> srcPts = {Point2f(320,190), 
								Point2f(445,190),
								Point2f(730,410),
								Point2f(85,410)};

const int sobelMinThreshold = 30;
const int sobelMaxThreshold = 255;

const int colorMinThreshold = 100;
const int colorMaxThreshold = 255;
//========================================================================
//================Trackbars===============================================
//========================================================================
#define PI 3.141592653589
#define BIRDEYE_VIEW 0
#define NORMAL_VIEW  1

int frameWidth = 640;
int frameHeight = 480;
int alpha_ = 10, beta_ = 90, gamma_ = 90;
int f_ = 746, dist_ = 450;
Mat I = imread("../data/virageD.png", CV_LOAD_IMAGE_COLOR);
Mat destination;
Size image_size = I.size();
//========================================================================
Mat calc()
{
	double focalLength, dist, alpha, beta, gamma; 
	alpha = ((double)alpha_-90) * PI/180;
	beta = ((double)beta_ -90) * PI/180;
	gamma = ((double)gamma_ -90) * PI/180;
	focalLength = (double)f_;
	dist = (double)dist_;
	double w = (double)image_size.width, h = (double)image_size.height;
	// Projecion matrix 2D -> 3D
	Mat A1 = (Mat_<float>(4, 3) << 1, 0, -w/2,
								   0, 1, -h/2,
								   0, 0, 0,
								   0, 0, 1 );
	// Rotation matrices Rx, Ry, Rz
	Mat RX = (Mat_<float>(4, 4) << 1, 0, 0, 0,
								  0, cos(alpha), -sin(alpha), 0,
								  0, sin(alpha), cos(alpha), 0,
								  0, 0, 0, 1 );
	Mat RY = (Mat_<float>(4, 4) << cos(beta), 0, -sin(beta), 0,
									0, 1, 0, 0,
									sin(beta), 0, cos(beta), 0,
									0, 0, 0, 1	);
	Mat RZ = (Mat_<float>(4, 4) << cos(gamma), -sin(gamma), 0, 0,
									sin(gamma), cos(gamma), 0, 0,
									0, 0, 1, 0,
									0, 0, 0, 1	);
	// R - rotation matrix
	Mat R = RX * RY * RZ;
	// T - translation matrix
	Mat T = (Mat_<float>(4, 4) << 1, 0, 0, 0,  
								  0, 1, 0, 0,  
								  0, 0, 1, dist,  
								  0, 0, 0, 1); 	
	// K - intrinsic matrix 
	Mat K = (Mat_<float>(3, 4) << focalLength, 0, w/2, 0,
								  0, focalLength, h/2, 0,
								  0, 0, 1, 0);
	Mat transformationMat = K * (T * (R * A1));	
	return transformationMat;
}

static void af(int, void*)
{
	Mat transformationM = calc();
	warpPerspective(I, destination, transformationM, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
	imshow("Result", destination);
}

static void bf(int, void*)
{
	Mat transformationM = calc();
	warpPerspective(I, destination, transformationM, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
	imshow("Result", destination);
}

static void gf(int, void*)
{
	Mat transformationM = calc();
	warpPerspective(I, destination, transformationM, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
	imshow("Result", destination);
}

static void ff(int, void*)
{
	Mat transformationM = calc();
	warpPerspective(I, destination, transformationM, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
	imshow("Result", destination);
}

static void df(int, void*)
{
	Mat transformationM = calc();
	warpPerspective(I, destination, transformationM, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
	imshow("Result", destination);
}
//========================================================================
//========================================================================
//========================================================================
void parameters()
{
	namedWindow("Result", CV_WINDOW_AUTOSIZE );
	namedWindow("Im_source", CV_WINDOW_AUTOSIZE );
	imshow("Im_source", I);
	//namedWindow("Result", CV_WINDOW_AUTOSIZE);
//Traitement
	resize(I, I, Size(frameWidth, frameHeight));
	//trackbar
	createTrackbar("Alpha"   , "Result", &alpha_, 180, af);
	createTrackbar("Beta"	, "Result", &beta_ , 180, bf);
	createTrackbar("Gamma"   , "Result", &gamma_, 180, gf);
	createTrackbar("f"	   , "Result", &f_	, 2000, ff);
	createTrackbar("Distance", "Result", &dist_ , 2000, df);
	af(0,0);
	bf(0,0);
	gf(0,0);
	ff(0,0);
	df(0,0);
	waitKey(0);
	destroyAllWindows();
}
//------------------------------------------------------------------------------
Mat sobelThresholding(Mat input, string dir="x")
{
	// Sobel Direction (x) and Thresholding
	// Params
	// Input : L Channel of HLS Img
	// Dir : Sobel Thresholding Direction . {X, Y}

	//Output
	//A Mat Img processed by Sobel direction thresholding between range sobelMinThreshold to sobelMaxThreshold
	Mat sobel;
	Mat sxbinary = Mat::zeros(input.size(), input.type());
	
	// Sobel X Gradient Thresholding
	if(dir == "x")
		Sobel(input, sobel, CV_64F, 1, 0);
	if(dir == "y")
		Sobel(input, sobel, CV_64F, 0, 1);
	
	convertScaleAbs(sobel, sobel);
	
	//Range based thresholding.
	//pixels between sobelMinThreshold and sobelMaxThreshold set to 255, otherwise 0
	inRange(sobel, Scalar(sobelMinThreshold), Scalar(sobelMaxThreshold), sxbinary);
	return sxbinary;		
}

Mat colorThresholding(Mat input)
{
	// Color Channel Thresholding	
	//Param
	//Input : S Channel of HLS Img.
	//Output
	//A Mat img processed by Color Thresholding Between
	//colorMinThreshold To colorMaxThreshold
	Mat s_binary = Mat::zeros(input.size(), input.type());	
	
	//Range based thresholding.
	//pixels between colorMinThreshold and colorMaxThreshold set to 255, otherwise 0
	inRange(input, Scalar(colorMinThreshold), Scalar(colorMaxThreshold), s_binary);
	return s_binary;
}

Mat sobelColorThresholding(Mat input)
{
	// Bitwise operation of sobel threshold img and color threshold img
	// Param
	// input :  Mat BGR Color Img
	// output
	// A Mat img with bitwise operation of sobel threshold img and color threshold img
	Mat sxbinary;
	Mat s_binary;
	Mat output;
	Mat splitedColor[3];
	
	//Convert Color from BGR to HLS channel
	//cvtColor(input, input, COLOR_BGR2HLS);
	
	//Split BGR Color spaces into B, G, R and store them in splitedColor array
	split(input, splitedColor);
	
	Mat imgForSobel = splitedColor[1].clone();
	Mat imgForColor = splitedColor[2].clone();
	
	//Apply sobelThresholding to G channel Image
	sxbinary = sobelThresholding(imgForSobel);
	
	//Apply colorThresholding to R channel Image
	s_binary = colorThresholding(imgForColor);
	
	//Bitwise operation to sum sobelThresholding and colorThresholding Images
	bitwise_and(sxbinary, s_binary, output);
	return output;   
}

void postProcess()
{
	Mat transformationM = calc();
	warpPerspective(I, destination, transformationM, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
	//Mat res1 = sobelColorThresholding(destination);
	Mat gray;
	cvtColor(destination, gray, CV_BGR2GRAY);
	imgUtils::computeHistogramme(gray, 1);
	int thresh = 120;
	int max_val = 255;
	Mat binary;
	//threshold(gray, binary, thresh, max_val, THRESH_BINARY);
	GaussianBlur(gray, binary, Size(17,17), 0);
	threshold(binary, binary, thresh, max_val, THRESH_BINARY);
	Mat erode_kernel = getStructuringElement(MORPH_RECT, Size(15, 15), Point(-1, -1));
	Mat im_erode;
	erode(binary, binary, erode_kernel);
	imshow("Bird view", destination);	
	imshow("Result", binary);
	imshow("src", I);
	waitKey(0);
	destroyAllWindows();
}

//========================================================================
int main(int argc, char **argv)
{
	help(argv);
	//parameters();
	postProcess();
	/*
	circle(I, srcPts[0], 10, CV_RGB(255,0,0));
	circle(I, srcPts[1], 10, CV_RGB(0,255,0));
	circle(I, srcPts[2], 10, CV_RGB(0,0,255));	
	circle(I, srcPts[3], 10, CV_RGB(255,255,255));
	*/

	return EXIT_SUCCESS;
}

static void help(char **argv) 
{
	cout << endl
		 << "This program demonstrated the use of the discrete Fourier transform (DFT). " << endl
		 << "The dft of an image is taken and it's power spectrum is displayed." << endl << endl
		 << "Usage:" << endl
		 << argv[0] << " [image_name -- default lena.jpg]" << endl << endl;
}


/*vector<Point2f> compute4Pts_Transform(vector<Point2f> src)
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
}*/