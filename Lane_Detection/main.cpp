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
	Mat I = imread("../data/virageG.png", CV_LOAD_IMAGE_COLOR);
	lane L(I);
	L.processFrame();
	cout << "Curve radius = " << L.m_curveRad << endl;
	cout << "Curve direction = " << L.m_curveDir << endl;
	cout << "OffCenter = " << L.m_offCenter << endl;		
	//imwrite("../data/BEV.png", L.m_BEV);
	imshow("Im_source", L.m_matSrc);
	imshow("Bird Eye View", L.m_BEV);
	waitKey(0);
	destroyAllWindows();	
	return EXIT_SUCCESS;
}
