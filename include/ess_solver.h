#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>
#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

#include <iostream>
#include <vector>

namespace ORB_SLAM2
{
class ess_solver{

	static bool computeFundamentalMatrixBy2Point( Mat *po1, Mat *po2, int *selected, Mat *essdata[10], Mat **R, Mat **S );
	static void computeFundamentalMatrixBy2Point( double r1, double r2, Mat *essdata[10], Mat **R, Mat **S );
	static void computeFundamentalMatrixBy5Point( Mat *po1, Mat *po2, int *selected, Mat *essdata[10], int &nofess );
	static void computeFundamentalMatrixBy5Point( char *name, char *name2  );

	static void init_5pt();
	static void exit_5pt();
};
}
