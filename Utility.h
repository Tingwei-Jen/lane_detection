#pragma once
#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class Utility
{
public:
	Utility();
	~Utility();

public:
	void polyfit(Eigen::VectorXd X, Eigen::VectorXd Y, int degree, Eigen::VectorXd &coeff);
	
	void polyfitRansac(Eigen::VectorXd X, Eigen::VectorXd Y, int degree, int iter_num, int sample_num, int img_height, int th_dist,  Eigen::VectorXd &coeff);

	void poly1d(Eigen::VectorXd coeff, double x, double &y);
	
	void trans2Eigen(std::vector<cv::Point> lane_pixs, Eigen::VectorXd& x, Eigen::VectorXd& y);

	void histogram(cv::Mat image, int n_bases, std::vector<int>& x_bases); 

public:
	double calCurvature(Eigen::VectorXd coeff);



};
#endif // UTILITY_H