#pragma once
#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "Utility.h"

struct parameter
{
	std::vector<cv::Point2f> _DynamicRoiPoints;
	int _NumberOfWindows;   
	int _MarginOfWindow;  
	int _MinpixInWindow; 
	int _IntervalOfRow;
 	int _LineWidthMax;
 	int _LineWidthMin;
 	int _AdativeROIBoundry;
 	int _MidOfView;
};

class LaneDetector
{
public:
	LaneDetector(parameter para);
	~LaneDetector();
	bool ProcessImage(cv::Mat image);
	std::vector<std::vector<cv::Point2f> > getLinePoints() { return mLinePoints; }
	std::vector<std::vector<cv::Point2f> > getLanePoints() { return mLanePoints; }
	std::vector<cv::Point2f> getDynamicROIPoints() { return mDynamicRoiPoints; }

public:
	enum eDetectState
	{
        NOT_INITIALIZED=0,
        DETECT_ONLY_LEFT=1,
        DETECT_ONLY_RIGHT=2,
        DETECTBOTH=3,
	};

	eDetectState mState;

private:
	void Update(cv::Mat input, bool verbose=false);

private:
	//IPM
	void Birdeye(cv::Mat input, cv::Mat &warped, cv::Mat &Minv, bool verbose=false);
	//get binary image about line
	void Binarize(cv::Mat warped, cv::Mat &binary, bool verbose=false);

	void LinePixsInWindow(cv::Mat binary, std::vector<std::vector<cv::Point> >& all_line_pixs, std::vector<int>& fitting_degree, bool verbose=false);

	void PredictLines(std::vector<std::vector<cv::Point> > all_line_pixs, std::vector<int> fitting_degree, 
							std::vector<std::vector<cv::Point2f> >& all_line_pixs_pred_ipm);


	void FindVanishPoint(std::vector<std::vector<cv::Point2f> > all_line_points, int image_height, 
									cv::Point& vp, std::vector<int>& all_bpxs, std::vector<cv::Point>& tentative_roi);
private:	
	void GenerateLanePoints(std::vector<std::vector<cv::Point2f> > all_line_points, std::vector<std::vector<cv::Point2f> >& all_lane_points);
	void FindBinaryThreshold(cv::Mat warped_gray, std::vector<int>& threshold);

	void UpdateROI(std::vector<std::vector<cv::Point2f>> all_line_pixs_pred_ipm);

private:
	Utility *utility;

private:
	std::vector<std::vector<cv::Point2f> > mLinePoints;   // all lines, from left to right
	std::vector<std::vector<cv::Point2f> > mLanePoints;   // all mid of lines, from left to right

	// dynamic roi in origin image
	//    -------      13
	//   /       \
	//  -----------    24
	std::vector<cv::Point2f> mDynamicRoiPoints;

	int mNumberOfWindows;   //number of sliding windows
	int mMarginOfWindow;    //width of the windows +/- margin
	int mMinpixInWindow;    //minimum number of pixels found to recenter window  
	int mIntervalOfRow;     //every how many rows to detect white pixs for reducing computation time
 	int mLineWidthMax;      //filtering white pixs but not lane
 	int mLineWidthMin;		//filtering white pixs but not lane
 	int mAdativeROIBoundry; //adaptive ROI region
 	int mMidOfView;
 	int mCount;
};
#endif // CURVED_LANE_DETECTOR_H