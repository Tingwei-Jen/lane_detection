#include "LaneDetector.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;

LaneDetector::LaneDetector(parameter para) : mState(NOT_INITIALIZED)
{
	cout<<"Constuct LaneDetector!"<<endl;
	this->mDynamicRoiPoints = para._DynamicRoiPoints;
	this->mNumberOfWindows = para._NumberOfWindows;   
	this->mMarginOfWindow = para._MarginOfWindow;  
	this->mMinpixInWindow = para._MinpixInWindow; 
	this->mIntervalOfRow = para._IntervalOfRow;
 	this->mLineWidthMax = para._LineWidthMax;
 	this->mLineWidthMin = para._LineWidthMin;
 	this->mAdativeROIBoundry = para._AdativeROIBoundry;
 	this->mMidOfView = para._MidOfView;
}

LaneDetector::~LaneDetector()
{
}

bool LaneDetector::ProcessImage(cv::Mat image)
{
	Update(image, true);     
	return true;
}

/*
private
*/
void LaneDetector::Update(cv::Mat input, bool verbose)
{
	cv::Mat warped, Minv;
	Birdeye(input, warped, Minv, verbose);                 

	cv::Mat binary;
	Binarize(warped, binary, verbose);

	std::vector<std::vector<cv::Point> > all_line_pixs;
	std::vector<int> fitting_degree;
	LinePixsInWindow(binary, all_line_pixs, fitting_degree, verbose);

	vector<vector<cv::Point2f>> all_line_pixs_pred_ipm;
	PredictLines(all_line_pixs, fitting_degree, all_line_pixs_pred_ipm);


	this->mLinePoints.clear();
	this->mLanePoints.clear();
	if(fitting_degree[0] == 0 && fitting_degree[1] == 0)
	{
		this->mState = NOT_INITIALIZED;
	} 
	else if (fitting_degree[0] != 0 && fitting_degree[1] == 0)
	{
		this->mState = DETECT_ONLY_LEFT;
		vector<cv::Point2f> line_pixs_pred_left;
		cv::perspectiveTransform(all_line_pixs_pred_ipm[0], line_pixs_pred_left, Minv);
		this->mLinePoints.push_back(line_pixs_pred_left);
	} 
	else if (fitting_degree[0] == 0 && fitting_degree[1] != 0)
	{
		this->mState = DETECT_ONLY_RIGHT;
		vector<cv::Point2f> line_pixs_pred_right;
		cv::perspectiveTransform(all_line_pixs_pred_ipm[1], line_pixs_pred_right, Minv);
		this->mLinePoints.push_back(line_pixs_pred_right);
	} 
	else 
	{
		this->mState = DETECTBOTH;
		vector<cv::Point2f> line_pixs_pred_left, line_pixs_pred_right;
		cv::perspectiveTransform(all_line_pixs_pred_ipm[0], line_pixs_pred_left, Minv);
		cv::perspectiveTransform(all_line_pixs_pred_ipm[1], line_pixs_pred_right, Minv);
		this->mLinePoints.push_back(line_pixs_pred_left);
		this->mLinePoints.push_back(line_pixs_pred_right);

		//generate lane
		GenerateLanePoints(this->mLinePoints, this->mLanePoints);

		//generate vanish point
		cv::Point vp;
		vector<int> bpxs;
		vector<cv::Point> tentative_roi;
		FindVanishPoint(this->mLinePoints, input.rows, vp, bpxs, tentative_roi);
		cv::circle(input, vp, 2, cv::Scalar(0,255,255), -1);

		for(int i=0; i<bpxs.size();i++)
			cv::line(input, vp, cv::Point(bpxs[i], input.rows), cv::Scalar(0, 0, 50*(i+1)), 1);
		
		for(int i=0; i<tentative_roi.size();i++)
				cv::circle(input, tentative_roi[i], 2, cv::Scalar(0,255,255), -1);
	
		//dynamic roi
		vector<cv::Point2f> dynamic_roi_pred;
		cout<<all_line_pixs_pred_ipm[0][0].x<<endl;
		cv::Point2f top_left(all_line_pixs_pred_ipm[0][0].x - this->mAdativeROIBoundry, 0);
		cv::Point2f buttom_left(all_line_pixs_pred_ipm[0][input.rows-1].x - this->mAdativeROIBoundry, input.rows);
		cv::Point2f top_right(all_line_pixs_pred_ipm[1][0].x + this->mAdativeROIBoundry, 0);
		cv::Point2f buttom_right(all_line_pixs_pred_ipm[1][input.rows-1].x + this->mAdativeROIBoundry, input.rows);

		dynamic_roi_pred.push_back(top_left); 
		dynamic_roi_pred.push_back(buttom_left); 
		dynamic_roi_pred.push_back(top_right); 
		dynamic_roi_pred.push_back(buttom_right); 

		cv::perspectiveTransform(dynamic_roi_pred, this->mDynamicRoiPoints, Minv);

		//switch lane judge!!!!







	}

	cout<<"this->mState: "<<this->mState<<endl;








	// if(verbose)
	// 	cv::imshow("regression line", plot);

	// //update ROI and find Lane points and Find vanishing point
	// if(this->mLinePoints.size()>=2)
	// {
	// 	cv::perspectiveTransform(roi_pixs_pred, this->mDynamicRoiPoints, Minv);
	// 	GenerateLanePoints(this->mLinePoints, this->mLanePoints);

	// 	cv::Point vp;
	// 	vector<int> bpxs;
	// 	vector<cv::Point> tentative_roi;
	// 	FindVanishPoint(this->mLinePoints, input.rows, vp, bpxs, tentative_roi);
		
	// 	cv::circle(input, vp, 2, cv::Scalar(0,255,255), -1);
		
	// 	for(int i=0; i<bpxs.size();i++)
	// 		cv::line(input, vp, cv::Point(bpxs[i], input.rows), cv::Scalar(0, 0, 50*(i+1)), 1);
		
	// 	for(int i=0; i<tentative_roi.size();i++)
	// 			cv::circle(input, tentative_roi[i], 2, cv::Scalar(0,255,255), -1);
	
	// 	for(int i=0; i<bpxs.size();i++)
	// 		cout<<bpxs[i]<<"    ";
	// 	cout<<endl;

	// 	//switching judge
	// 	if(this->mSwitchState == NO_SWITCH)
	// 	{
	// 		if(this->mMidOfView < (bpxs[1]+(bpxs[2]-bpxs[1])/4))
	// 		{
	// 			cout<<"Expand to left"<<endl;
	// 			this->mSwitchState = SWITCH_LEFT;
	// 			this->mDrawLaneInExpandROI = true;
	// 			this->mDynamicRoiPoints[0].x = tentative_roi[0].x - 15;
	// 			this->mDynamicRoiPoints[1].x = tentative_roi[1].x -35;		
	// 		} else if (this->mMidOfView > (bpxs[1]+(bpxs[2]-bpxs[1])*3/4))
	// 		{
	// 			cout<<"Expand to right"<<endl;
	// 			this->mSwitchState = SWITCH_RIGHT;
	// 			this->mDrawLaneInExpandROI = true;
	// 			this->mDynamicRoiPoints[2].x = tentative_roi[2].x + 15;
	// 			this->mDynamicRoiPoints[3].x = tentative_roi[3].x + 35;	
	// 		}
	// 	}
	// 	else if (this->mSwitchState == SWITCH_LEFT && this->mMidOfView < (bpxs[1]+(bpxs[2]-bpxs[1])*3/4)) 
	// 	{
	// 			cout<<"finish switch"<<endl;
	// 			cout<<bpxs[1]<<"  "<<bpxs[2]<<endl;
	// 			this->mSwitchState = NO_SWITCH;
	// 			this->mDrawLaneInExpandROI = false;
	// 			this->mDynamicRoiPoints[2].x = this->mLinePoints[1][0].x + 15;
	// 			this->mDynamicRoiPoints[3].x = this->mLinePoints[1][this->mLinePoints[1].size()-1].x + 35;
	// 	}
	// 	else if (this->mSwitchState == SWITCH_RIGHT && this->mMidOfView > (bpxs[2]+(bpxs[3]-bpxs[2])/4)) 
	// 	{
	// 			cout<<"finish switch"<<endl;
	// 			this->mSwitchState = NO_SWITCH;
	// 			this->mDrawLaneInExpandROI = false;
	// 			this->mDynamicRoiPoints[0].x = this->mLinePoints[1][0].x - 15;
	// 			this->mDynamicRoiPoints[1].x = this->mLinePoints[1][this->mLinePoints[1].size()-1].x - 35;
	// 	}
	// }

	// this->mCount++;
}


/*
Do IPM and get transform matrix
Output: BGR image, and inverse matrix
*/
void LaneDetector::Birdeye(cv::Mat input, cv::Mat &warped, cv::Mat &Minv, bool verbose)
{
	// Input Quadilateral or Image plane coordinates
	cv::Point2f inputQuad[4];
	// Output Quadilateral or World plane coordinates
	cv::Point2f outputQuad[4];
	// M Matrix
	cv::Mat M(3, 3, CV_32FC1);

	// The 4 points that select quadilateral on the input , from bottom-left in clockwise order
	// These four pts are the sides of the rect box used as input 
	inputQuad[0] = this->mDynamicRoiPoints[0];
	inputQuad[1] = this->mDynamicRoiPoints[1];
	inputQuad[2] = this->mDynamicRoiPoints[2];
	inputQuad[3] = this->mDynamicRoiPoints[3];

	outputQuad[0] = cv::Point2f(0, 0);
	outputQuad[1] = cv::Point2f(0, input.rows);
	outputQuad[2] = cv::Point2f(input.cols, 0);
	outputQuad[3] = cv::Point2f(input.cols, input.rows);

	// Get the Perspective Transform Matrix
	M = cv::getPerspectiveTransform(inputQuad, outputQuad);
	Minv = cv::getPerspectiveTransform(outputQuad, inputQuad);

	// Apply the Perspective Transform just found to the src image
	cv::warpPerspective(input, warped, M, input.size());

	if(verbose)
	{
		cv::Mat ROI = input.clone(); 
		cv::line(ROI, cv::Point(int(inputQuad[0].x), int(inputQuad[0].y)), cv::Point(int(inputQuad[1].x), int(inputQuad[1].y)), cv::Scalar(0, 97, 255), 2);
		cv::line(ROI, cv::Point(int(inputQuad[1].x), int(inputQuad[1].y)), cv::Point(int(inputQuad[3].x), int(inputQuad[3].y)), cv::Scalar(0, 97, 255), 2);
		cv::line(ROI, cv::Point(int(inputQuad[3].x), int(inputQuad[3].y)), cv::Point(int(inputQuad[2].x), int(inputQuad[2].y)), cv::Scalar(0, 97, 255), 2);
		cv::line(ROI, cv::Point(int(inputQuad[2].x), int(inputQuad[2].y)), cv::Point(int(inputQuad[0].x), int(inputQuad[0].y)), cv::Scalar(0, 97, 255), 2);
		
		cv::imshow("ROI", ROI);
		cv::imshow("warped", warped);
	}
}

/*
input: warped bgr image
output: binary image
*/
void LaneDetector::Binarize(cv::Mat warped, cv::Mat &binary, bool verbose)
{
	cv::Mat warped_gray;
	cv::cvtColor(warped, warped_gray, CV_BGR2GRAY);

	vector<int> grayscale_threshold;	
	FindBinaryThreshold(warped_gray, grayscale_threshold);

	// for(int i=0; i<grayscale_threshold.size(); i++)
	// 	cout<<grayscale_threshold[i]<<endl;

	int num_x_part = 2;
	int num_y_part = 2;

	int width = warped_gray.cols/num_x_part;
	int height = warped_gray.rows/num_y_part;

	vector<cv::Mat> all_mask_temp;
	int index = 0;

	for(int x=0; x<num_x_part; x++)
	{
		for(int y=0; y<num_y_part; y++)
		{
			cv::Mat temp = cv::Mat::zeros(warped_gray.rows, warped_gray.cols, warped_gray.type());

			for(int col=width*x; col<width*(x+1); col++)
			{
				for(int row=height*y; row<height*(y+1); row++)
				{
					temp.at<uchar>(row, col) = warped_gray.at<uchar>(row, col);					
				}
			}

			cv::Mat mask_temp;
			cv::threshold(temp, mask_temp, grayscale_threshold[index], 255, cv::THRESH_BINARY);
			index++;
			all_mask_temp.push_back(mask_temp);
		}
	}

	cv::Mat color_mask = cv::Mat::zeros(warped_gray.rows, warped_gray.cols, warped_gray.type());
	
	for(int i=0; i<all_mask_temp.size(); i++)
		color_mask = color_mask+all_mask_temp[i];

	//Erosion and Dilation
    cv::Mat erodeStruct = getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(color_mask, binary, cv::MORPH_CLOSE, erodeStruct);

	if(verbose)
	{	
		// cout<<"grayscale_threshold: ";
		// for(int i=0; i<grayscale_threshold.size(); i++)
		// 	cout<<grayscale_threshold[i]<<"   ";
		// cout<<endl;
		
		cv::imshow("warped-gray", warped_gray);
		cv::imshow("binary", binary);
	}
}


void LaneDetector::LinePixsInWindow(cv::Mat binary, std::vector<std::vector<cv::Point> >& all_line_pixs, 
	 														std::vector<int>& fitting_degree, bool verbose)
{
	int n_lines = 2;

	cv::Mat plot;
	cv::cvtColor(binary, plot, cv::COLOR_GRAY2BGR);

	int height = binary.rows;
 	int width = binary.cols;

 	all_line_pixs.clear();
 	fitting_degree.clear();

 	// height of each sliding window
	int window_height = int(height/this->mNumberOfWindows);

	//from left to right
	std::vector<int> x_bases;   
	utility->histogram(binary, n_lines, x_bases);

	//update x current every window
	std::vector<int> x_current;
	for(int i=0; i<n_lines; i++)
		x_current.push_back(x_bases[i]);

	for(int line=0; line<n_lines; line++)
	{
		//for deciding fitting degree
		int idx_win_max = 0;
		int idx_win_min = this->mNumberOfWindows;

		std::vector<cv::Point> line_pixs;

		for(int window = 0; window<this->mNumberOfWindows; window++)
		{
			//window's boundry
	    	int win_y_low = height - (window + 1) * window_height;
	        int win_y_high = height - window * window_height;
	        int win_x_low = x_current[line] - this->mMarginOfWindow;
	        int win_x_high = x_current[line] + this->mMarginOfWindow;

	        if( win_x_low < 0 )
	        	win_x_low = 0;
	        if( win_x_high > width )
	        	win_x_high = width;

	        if( verbose )
	        	cv::rectangle(plot, cv::Point(win_x_low, win_y_low), cv::Point(win_x_high, win_y_high), cv::Scalar(0,0,255), 2);

	        //record pixs in window
	        vector<cv::Point> good_line_pixs;
	        for( int row = win_y_low; row < win_y_high; row = row + this->mIntervalOfRow )
	        {
	        	//avoiding white pixs but not lines
				int x_min = width;
				int x_max = 0;

				for( int col = win_x_low; col < win_x_high; col++ )
				{
					if( binary.at<uchar>(row,col) == 255 )
					{
						if( col < x_min )
							x_min = col;
						if( col > x_max )
							x_max = col;
					}
				}

				int line_width = x_max - x_min;	

				// record white pixs and avoid not line but white pixel
				if( line_width > this->mLineWidthMin && line_width < this->mLineWidthMax )
				{
					if(idx_win_min > window)
						idx_win_min = window;
					if(idx_win_max < window)
						idx_win_max = window;

					for( int col = x_min; col < x_max; col++ )
					{
						good_line_pixs.push_back(cv::Point(col,row));
						if( verbose )
							cv::circle(plot, cv::Point(col,row), 1, cv::Scalar(255,255,0), -1);
					}
				}
	        }

	        //update x_current
	        if( good_line_pixs.size() > this->mMinpixInWindow )
			{
				int sum = 0;
	        	for( int i=0; i<good_line_pixs.size(); i++ )
					sum = sum + good_line_pixs[i].x;
		
				x_current[line] = int(sum/good_line_pixs.size());
	        }
	        
	        //record line_pixs and 
	        for( int i=0; i<good_line_pixs.size(); i++ )
				line_pixs.push_back(good_line_pixs[i]);
		}

		all_line_pixs.push_back(line_pixs);

		//decide fitting degree
		if((idx_win_max - idx_win_min)>5)
			fitting_degree.push_back(2);
		else if ((idx_win_max - idx_win_min)>1 && (idx_win_max - idx_win_min)<=5)
			fitting_degree.push_back(1);
		else 
			fitting_degree.push_back(0);
	}

	if(verbose)
		cv::imshow("sliding_window",plot);
}

void LaneDetector::PredictLines(vector<vector<cv::Point> > all_line_pixs, vector<int> fitting_degree, 
														vector<vector<cv::Point2f> >& all_line_pixs_pred_ipm)
{
	all_line_pixs_pred_ipm.clear();
	int height = 480;
	int n_lines = 2;

	//predict lines
	for(int i=0; i<n_lines; i++)
	{
		Eigen::VectorXd line_pixs_x, line_pixs_y, fitting_coeff;
		utility->trans2Eigen(all_line_pixs[i], line_pixs_x, line_pixs_y);
		utility->polyfit(line_pixs_y, line_pixs_x, fitting_degree[i], fitting_coeff);  

		vector<cv::Point2f> line_pixs_pred_ipm;

		for( int row = 0; row < height; row++ )
		{
			double x_predict;
			utility->poly1d(fitting_coeff, row, x_predict);
			line_pixs_pred_ipm.push_back(cv::Point2f(x_predict, row));
		
		}
		all_line_pixs_pred_ipm.push_back(line_pixs_pred_ipm);
	}
}

//x-x1 = m*(y-y1)
//x-my = x1-m*y1
void LaneDetector::FindVanishPoint(std::vector<std::vector<cv::Point2f> > all_line_points, int image_height, 
														cv::Point& vp, std::vector<int>& all_bpxs, vector<cv::Point>& tentative_roi)
{
	all_bpxs.clear();
	tentative_roi.clear();

	int n_lines = all_line_points.size();

	//step1: find VP 
	//x-x1 = m*(y-y1)
	//x-my = x1-m*y1
	//step2: Base Point
	//x = m1y + top.x-m1*top.y
	//set y = 480
	Eigen::MatrixXd A(n_lines,2);
	Eigen::MatrixXd B(n_lines,1);

	std::vector<int> bpxs;

	for(int i=0; i<n_lines;i++)
	{
		cv::Point top = all_line_points[i][0]; 
		cv::Point buttom = all_line_points[i][all_line_points[i].size()-1]; 

		float m;
		m = float(top.x-buttom.x)/float(top.y-buttom.y);

		A(i,0) = 1; A(i,1) = -m;
		B(i,0) = top.x-m*top.y;

		int bp = int( m*image_height + top.x-m*top.y);
		bpxs.push_back(bp);
	}
	
	Eigen::MatrixXd AtA = A.transpose()*A;
	Eigen::VectorXd _vp = AtA.inverse()*A.transpose()*B;

	vp.x = _vp(0);
	vp.y = _vp(1);

	//step3: find two assistant lines
	int sum = 0;
	for(int i=0; i<bpxs.size()-1; i++)
		sum = sum + (bpxs[i+1]-bpxs[i]);
	
	int range = sum/(bpxs.size()-1);

	int bpx_left = bpxs[0]-range;
	int bpx_right = bpxs[bpxs.size()-1]+range;

	all_bpxs.push_back(bpx_left);

	for(int i=0; i<bpxs.size(); i++)
		all_bpxs.push_back(bpxs[i]);

	all_bpxs.push_back(bpx_right);

	//step4: find other two line equation and tentative roi points
	// x-m3y = vpx-m3*vpy
	// x = m3*Roi_buttom + vpx - m3*vpy
	float m1, m2;
	m1 = float(_vp(0)-bpx_left)/float(_vp(1)-image_height);
	m2 = float(_vp(0)-bpx_right)/float(_vp(1)-image_height);

	int xleft_top, xleft_buttom, xright_top, xright_buttom;

	xleft_top = int(m1*this->mDynamicRoiPoints[0].y + _vp(0) - m1*_vp(1));
	xleft_buttom = int(m1*this->mDynamicRoiPoints[1].y + _vp(0) - m1*_vp(1));
	xright_top = int(m2*this->mDynamicRoiPoints[2].y + _vp(0) - m2*_vp(1));
	xright_buttom = int(m2*this->mDynamicRoiPoints[3].y + _vp(0) - m2*_vp(1));
	
	tentative_roi.push_back(cv::Point(xleft_top, this->mDynamicRoiPoints[0].y));
	tentative_roi.push_back(cv::Point(xleft_buttom, this->mDynamicRoiPoints[1].y));
	tentative_roi.push_back(cv::Point(xright_top, this->mDynamicRoiPoints[2].y));
	tentative_roi.push_back(cv::Point(xright_buttom, this->mDynamicRoiPoints[3].y));
}

void LaneDetector::GenerateLanePoints(vector<vector<cv::Point2f>> all_line_points, vector<vector<cv::Point2f>>& all_lane_points)
{
	all_lane_points.clear();
	int n_lines = all_line_points.size();
	int n_lanes = n_lines-1;

	int n_line_points = all_line_points[0].size();

	for(int i=0; i<n_lanes; i++)
	{
		vector<cv::Point2f> lane_points;
		for(int j=0; j<n_line_points; j++)
		{
			float x = (all_line_points[i][j].x + all_line_points[i+1][j].x) / 2;
			float y = (all_line_points[i][j].y + all_line_points[i+1][j].y) / 2;
			lane_points.push_back(cv::Point2f(x,y));
		}
		all_lane_points.push_back(lane_points);
	}
}

/*
seperate to 4 part then calculate grayscale threshold each part
---------
| 1 | 3 |
-------
| 2 | 4 |
---------
*/
void LaneDetector::FindBinaryThreshold(cv::Mat warped_gray, std::vector<int>& threshold)
{
	cv::Mat temp = warped_gray.clone();
	threshold.clear();

	int num_x_part = 2;
	int num_y_part = 2;

	int width = warped_gray.cols/num_x_part; 
	int height = warped_gray.rows/num_y_part;		

	int col_gap = 6;
	int row_gap = 3;

	int interval = 10;
	int number_interval = 255/interval;

	int mininum_grayscale = 95;
	int boundry = mininum_grayscale/interval;
	int value_gap = 15;

	for(int x=0; x<num_x_part; x++)
	{
		for(int y=0; y<num_y_part; y++)
		{
			vector<int> warped_gray_value;

			for(int col=width*x; col<width*(x+1); col=col+col_gap)
			{
				for(int row=height*y; row<height*(y+1); row=row+row_gap)
				{

					warped_gray_value.push_back(temp.at<uchar>(row,col));
					temp.at<uchar>(row,col) = 255;
				}
			}

			int number[number_interval] = {0};

			for(int i=0; i<warped_gray_value.size(); i++)
			{
				int s = warped_gray_value[i]/interval;
				number[s]++;
			}

			int max_idx_non_zero = number_interval;

			for(int i=number_interval; i>=boundry; i--)
			{
				if(number[i]!=0 && number[i]>=10)
				{
					max_idx_non_zero = i;
					break;
				}
			}

			threshold.push_back(max_idx_non_zero*interval - value_gap);	
		}
	}
}