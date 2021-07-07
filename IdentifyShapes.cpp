// IdentifyShapes.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

// find the maximum cosine of the angle between joint edges
// law of cos
inline double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

int main()
{
	cv::VideoCapture main("C:/Users/Lance/source/repos/IdentifyShapes/x64/Debug/shapes.mp4");
	//cv::VideoCapture main(0);

	int lH = 3, lS = 3, lV = 3, hH = 179, hS = 255, hV = 255;
	std::string trackBarWindowName = "Trackbars";
	cv::namedWindow(trackBarWindowName);
	cv::createTrackbar("Lower Hue", trackBarWindowName, &lH, 179);
	cv::createTrackbar("Lower Saturation", trackBarWindowName, &lS, 255);
	cv::createTrackbar("Lower Value", trackBarWindowName, &lV, 255);
	cv::createTrackbar("Higher Hue", trackBarWindowName, &hH, 179);
	cv::createTrackbar("Higher Saturation", trackBarWindowName, &hS, 255);
	cv::createTrackbar("Higher Value", trackBarWindowName, &hV, 255);
	
	while(true)
	{
		std::vector<std::vector<Point>> triangles;
		std::vector<std::vector<Point>> rectangles;
		std::vector<std::vector<Point>> hexagons;
		
		//loop the video
		if (main.get(cv::CAP_PROP_POS_FRAMES) == main.get(cv::CAP_PROP_FRAME_COUNT))
		{
			main.set(cv::CAP_PROP_POS_FRAMES, 0);
		}

		//escape key breaks out of loop, wont run otherwise because then the loop runs forewver
		if (cv::waitKey(1) == 27)
		{
			break;
		}
		cv::Mat mainMat;
		main.read(mainMat);

		cv::Mat HSVMat;
		cv::cvtColor(mainMat, HSVMat, cv::COLOR_BGR2HSV);
		cv::Mat threshMat;
		//hue, saturation, value, value how dark/white a color is, saturation how vivid, hue what color
		cv::inRange(HSVMat, cv::Scalar(lH, lS, lV), cv::Scalar(hH, hS, hV), threshMat);

		//morpohloical transfoormations
		cv::erode(threshMat, threshMat, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
		cv::erode(threshMat, threshMat, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
		cv::dilate(threshMat, threshMat, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
		cv::dilate(threshMat, threshMat, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
		
		cv::imshow("thresholded", threshMat);
		//basically points of objects
		std::vector<std::vector<Point>> contours;
		std::vector<Point> approx;
		Mat contoursMat;
		//hierarchy not important
		std::vector<Vec4i> hierarchy;
		cv::findContours(threshMat, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		cv::drawContours(contoursMat, contours, -1, Scalar(0, 255, 0), 3, LINE_AA, hierarchy, 3);
		for (size_t i = 0; i < contours.size(); i++) 
		{
			approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
			if (fabs(contourArea(approx)) > 1000 && isContourConvex(approx))
			{
				double maxCosine = 0;
				double maxAngle = 0;
				switch (approx.size())
				{
					case 3:
						for (int j = 2; j < 4; j++)
						{
							// find the maximum cosine of the angle between joint edges
							double cosine = fabs(angle(approx[j % 3], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
						}
					
						maxAngle = acos(maxCosine) * (180 / 3.1415);
						// if the maxAngle is ~60 degrees push back because triangles have around 90 degrees for each vertice
						//dont need to check if the min angle is less because we know there are 3 points and triangles at least have 270 degrees
						if (maxAngle < 70)
						{
							triangles.push_back(approx);
						}
					
						break;
					case 4:
						for (int j = 2; j < 5; j++)
						{
							// find the maximum cosine of the angle between joint edges
							double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
						}

						//cos to angle for readability
						maxAngle = acos(maxCosine) * (180 / 3.1415);
						// if the maxAngle is ~90 degrees push back
						//dont need to check if the min angle is less because we know there are four points and quadlaterials at least have 360 degrees
						if (maxAngle < 100)
						{
							rectangles.push_back(approx);
						}
						break;
					
					case 6:
						for (int j = 2; j < 7; j++)
						{
							// find the maximum cosine of the angle between joint edges
							double cosine = fabs(angle(approx[j % 5], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
						}

						maxAngle = acos(maxCosine) * (180 / 3.1415);
						// if the maxAngle is ~60 degrees push back because triangles have around 90 degrees for each vertice
						//dont need to check if the min angle is less because we know there are 3 points and triangles at least have 270 degrees
						if (maxAngle < 130)
						{
							hexagons.push_back(approx);
						}
						break;
					default:
						break;
				}

			}
		}

		polylines(mainMat, triangles, true, Scalar(255, 0, 0), 2, LINE_AA);
		polylines(mainMat, rectangles, true, Scalar(0, 255, 0), 2, LINE_AA);
		polylines(mainMat, hexagons, true, Scalar(0, 0, 255),2, LINE_AA);

		for (std::vector<Point> s : triangles)
		{
			Rect rect = boundingRect(s);
			rectangle(mainMat, rect, Scalar(255, 0, 0), 2, LINE_AA);
			putText(mainMat, "triangle", rect.tl() - Point(0, 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 1, LINE_AA);
		}

		for (std::vector<Point> s : rectangles)
		{
			Rect rect = boundingRect(s);
			rectangle(mainMat, rect, Scalar(0, 255, 0), 2, LINE_AA);
			putText(mainMat, "rectangle", rect.tl() - Point(0, 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 1, LINE_AA);
		}

		for (std::vector<Point> s : hexagons)
		{
			Rect rect = boundingRect(s);
			rectangle(mainMat, rect, Scalar(0, 0, 255), 2, LINE_AA);
			putText(mainMat, "hexagon", rect.tl() - Point(0, 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, LINE_AA);
		}
		
		cv::imshow("Video Displays", mainMat);
	}
	return 0;
}
