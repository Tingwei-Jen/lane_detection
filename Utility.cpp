#include <iostream>
#include <cstdlib> /* 亂數相關函數 */
#include <ctime>   /* 時間相關函數 */
#include "Utility.h"

using namespace std;

Utility::Utility()
{
	cout<<"Construct Utility"<<endl;
}

void Utility::polyfit(Eigen::VectorXd X, Eigen::VectorXd Y, int degree, Eigen::VectorXd &coeff)
{
	int size = X.size();
	Eigen::MatrixXd A(size,degree+1);

	for(int i=0; i<A.rows();i++)
	{
		for(int j=0; j<A.cols();j++)
		{
			A(i,j) = pow(X(i),j);
		}
	}

	Eigen::MatrixXd AtA = A.transpose()*A;
	coeff = AtA.inverse()*A.transpose()*Y;
}

void Utility::polyfitRansac(Eigen::VectorXd X, Eigen::VectorXd Y, int degree, int iter_num, int sample_num, int img_height, int th_dist, Eigen::VectorXd &coeff)
{

	cv::Mat ransac = cv::Mat::zeros(480, 640, CV_8UC3);

	int size = X.size();
	srand( time(NULL) );
	int inliers_num_max = 0;

	for(int i=0; i<iter_num; i++)
	{
		vector<int> sample_idx;
		vector<int>::iterator it;
		for(int j=0; j<sample_num; j++)
		{
			int x = rand() % size;
			it = find (sample_idx.begin(), sample_idx.end(), x);  // avoid repeat
			if (it == sample_idx.end())
				sample_idx.push_back(x);
		}

		Eigen::VectorXd sampleX(sample_idx.size());
		Eigen::VectorXd sampleY(sample_idx.size());
		
		for(int j=0; j<sample_idx.size(); j++)
		{
			sampleX(j) = X(sample_idx[j]);
			sampleY(j) = Y(sample_idx[j]);

			cv::circle(ransac, cv::Point(sampleY(j),sampleX(j)), 2, cv::Scalar(0,255,255), -1);
		}
		
		Eigen::VectorXd _coeff;
		polyfit(sampleX, sampleY, degree, _coeff);    // generate fitting line according to samples
		vector<cv::Point> predict_line;

		int inliers_num = 0;
		for(int j=0; j<size; j++)
		{
			double predict;
			poly1d(_coeff, X(j), predict);
			if( abs(Y(j)-predict) < th_dist ) //inliers
			{
				inliers_num++;
				cv::circle(ransac, cv::Point(Y(j), X(j)), 2, cv::Scalar(255,0,255), -1);
			}	
		}

		if(inliers_num_max < inliers_num)
		{
			inliers_num_max = inliers_num;
			coeff = _coeff;
		}
	}


	cv::imshow("ransac", ransac);
}



void Utility::poly1d(Eigen::VectorXd coeff, double x, double &y)
{
	int size = coeff.size();
	Eigen::VectorXd X(size);

	for(int i=0; i<size; i++)
	{
		X(i) = pow(x,i);
	}
	
	y = X.dot(coeff);
}

void Utility::trans2Eigen(std::vector<cv::Point> lane_pixs, Eigen::VectorXd& x, Eigen::VectorXd& y)
{
	x = Eigen::VectorXd(lane_pixs.size());
	y = Eigen::VectorXd(lane_pixs.size());

    for( int i = 0; i < lane_pixs.size(); i++ )
    {
    	x(i) = lane_pixs[i].x;
    	y(i) = lane_pixs[i].y;
    }
}

/* 
get histogram of binary image in birdeye image and buttom location of lines 
*/
void Utility::histogram(cv::Mat image, int n_bases, std::vector<int>& x_bases)
{

	int height = image.rows;
 	int width = image.cols;

	vector<int> histogram; 

	//number of white pixels
	int n_white_pixs;

	//get histogram
	for(int col=0; col<width; col++)
	{
		n_white_pixs = 0;

		for(int row=0; row<height; row++)
		{
			if(image.at<uchar>(row,col) !=0)
				n_white_pixs++;
		}
		histogram.push_back(n_white_pixs);
	}

	int interval = width/n_bases;
	x_bases.clear();

	for( int i=0; i<n_bases;i++ )
	{
	    int x_low = i*interval;
	    int x_high = (i+1) * interval;
	    int pix_max = 0;
	    int x_base = x_low;

	    for( int col=x_low; col<x_high; col++ )
	    {
			if(pix_max<histogram[col])
			{
				pix_max = histogram[col];
				x_base = col;
			}
	    }
	    x_bases.push_back(x_base);
	}
}


/*
Calculate curvature of 2 degree polynomial function
y = c0+c1x+c2x2
k = abs(y")/pow((1+y'2),3/2)
y' = c1+2*c2*x
y" = 2*c2 
*/
double Utility::calCurvature(Eigen::VectorXd coeff)
{
	double c0 = coeff[0];
	double c1 = coeff[1];
	double c2 = coeff[2];

	//x: row = 0 
	double yd = c1+2*c2*0;
	double yd2 = yd*yd;
	double ydd = 2*c2;

	double curvature = abs(ydd)/pow((1+yd2),1.5);

	return curvature;
}