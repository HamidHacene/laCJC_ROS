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
#include "xtensor/xfixed.hpp"
#include "xtensor-blas/xlinalg.hpp"
//=============================
#include <chrono> 
using namespace std::chrono; 


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
	auto start = high_resolution_clock::now(); 
	
	Mat I = imread("../data/5.png", CV_LOAD_IMAGE_COLOR);
	lane L(I);
	L.processFrame();
	L.buildVisu("Result");
	
	auto stop = high_resolution_clock::now();
	
	auto duration = duration_cast<microseconds>(stop - start);
	cout << "elapsed time : " << duration.count()  << " microseconds" << endl;
	//imwrite("../data/BEV.png", L.m_BEV);
	
	//imshow("Result", visual);
	//imshow("Im_source", L.m_matSrc);
	//imshow("Bird Eye View", L.m_BEV);
	
	waitKey(0);
	destroyAllWindows();	
	return EXIT_SUCCESS;
}

