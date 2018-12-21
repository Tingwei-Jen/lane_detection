#include "LaneDetector.h"
#include <ctime>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;
using namespace Eigen;


void setting(parameter& para)
{
	//curve
	// para._DynamicRoiPoints.push_back(cv::Point2f(320.0, 325.0));
	// para._DynamicRoiPoints.push_back(cv::Point2f(180.0, 380.0));
	// para._DynamicRoiPoints.push_back(cv::Point2f(460.0, 325.0));
	// para._DynamicRoiPoints.push_back(cv::Point2f(520.0, 380.0));
	//curvetoright
	// para._DynamicRoiPoints.push_back(cv::Point2f(302.0, 310.0));
	// para._DynamicRoiPoints.push_back(cv::Point2f(180.0, 380.0));
	// para._DynamicRoiPoints.push_back(cv::Point2f(390.0, 310.0));
	// para._DynamicRoiPoints.push_back(cv::Point2f(513.0, 380.0));
	
	//test5
	para._DynamicRoiPoints.push_back(cv::Point2f(230.0, 270.0));
	para._DynamicRoiPoints.push_back(cv::Point2f(130.0, 380.0));
	para._DynamicRoiPoints.push_back(cv::Point2f(350.0, 270.0));
	para._DynamicRoiPoints.push_back(cv::Point2f(500.0, 380.0));
	
	para._NumberOfWindows = 9;
	para._MarginOfWindow = 40;  
	para._MinpixInWindow = 50; 
	para._IntervalOfRow = 5;
 	para._LineWidthMax = 30;
 	para._LineWidthMin = 5; 
 	para._AdativeROIBoundry = 40;
 	para._MidOfView = 280;
}

void imagetest();



int main() 
{
	//imagetest();
	parameter para;
	setting(para);
	LaneDetector *lanedetector = new LaneDetector(para);
                               
	VideoCapture cap("../data/test5/test5-1.mp4");
	if (!cap.isOpened()) {
	 	cout << "Error opening video stream or file" << endl;
	 	waitKey(0);
	}

	// VideoWriter writer;
	// Size videoSize =  cv::Size(640, 480);
 	// writer.open("temp.avi", CV_FOURCC('D', 'I', 'V', '3'), 30, videoSize);

	int index = 100;
	while (1) {

	 	Mat frame;
	 	cap >> frame;
	
	 	if (frame.empty())
	 		break;
		
		//Mat srcROI( frame, Rect(160,160,950,390));
	 	//cv::resize(srcROI, srcROI, cv::Size(640, 480));
		
		cv::resize(frame, frame, cv::Size(640, 480));
	 	
	 	// string filename1;
	 	// filename1 = "../data/test5/inputs/frame" + std::to_string(index) + ".jpg";
	 	// imwrite(filename1, frame);
		
	 	std::clock_t start;
	 	double duration;
	 	start = std::clock();

	 	lanedetector->ProcessImage(frame);

	 	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	 	std::cout << "fps: " << 1/duration << '\n';



	 	//output:
	 	vector<Point2f> DynamicRoiPoints;
		DynamicRoiPoints = lanedetector->getDynamicROIPoints();

		std::vector<std::vector<cv::Point2f> > LinePoints;
		LinePoints = lanedetector->getLinePoints();

		std::vector<std::vector<cv::Point2f> > LanePoints;
		LanePoints = lanedetector->getLanePoints();

		cv::line(frame, cv::Point(int(DynamicRoiPoints[0].x), int(DynamicRoiPoints[0].y)), cv::Point(int(DynamicRoiPoints[1].x), int(DynamicRoiPoints[1].y)), cv::Scalar(255, 0, 0), 3);
		cv::line(frame, cv::Point(int(DynamicRoiPoints[1].x), int(DynamicRoiPoints[1].y)), cv::Point(int(DynamicRoiPoints[3].x), int(DynamicRoiPoints[3].y)), cv::Scalar(255, 0, 0), 3);
		cv::line(frame, cv::Point(int(DynamicRoiPoints[3].x), int(DynamicRoiPoints[3].y)), cv::Point(int(DynamicRoiPoints[2].x), int(DynamicRoiPoints[2].y)), cv::Scalar(255, 0, 0), 3);
		cv::line(frame, cv::Point(int(DynamicRoiPoints[2].x), int(DynamicRoiPoints[2].y)), cv::Point(int(DynamicRoiPoints[0].x), int(DynamicRoiPoints[0].y)), cv::Scalar(255, 0, 0), 3);

		cv::line(frame, cv::Point(para._MidOfView , 450), cv::Point(para._MidOfView , 480), cv::Scalar(255, 255, 0), 3);

		for(int i=0; i<LinePoints.size();i++)
		{
			for(int j=0; j<LinePoints[i].size(); j++)
	 			cv::circle(frame, Point(LinePoints[i][j].x, LinePoints[i][j].y), 1, Scalar(0,255,0), -1);
		}

		for(int i=0; i<LanePoints.size();i++)
		{
			for(int j=0; j<LanePoints[i].size(); j++)
	 			cv::circle(frame, Point(LanePoints[i][j].x, LanePoints[i][j].y), 1, Scalar(0,255,255), -1);
		}

	 	imshow("frame", frame);
	 	//writer.write(srcROI);
	 	


	 	// string filename2;
	 	// filename2 = "../data/test5/outputs/frame_with_lines" + std::to_string(index) + ".jpg";
	 	// imwrite(filename2, srcROI);


		waitKey(0);
	 	index++;
	}

	cap.release();
	destroyAllWindows();
	waitKey(0);

	return 0;
}

void imagetest()
{
	Mat image;
    image = imread("../data/test6/frame2.jpg", CV_LOAD_IMAGE_COLOR);  
	cv::resize(image, image, cv::Size(640, 480));
    
    parameter para;
	setting(para);
	LaneDetector *lanedetector = new LaneDetector(para);
	lanedetector->ProcessImage(image);
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", image );                  
    waitKey(0);                                         
}


