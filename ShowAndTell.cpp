// ShowAndTell.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

//cosine from three points of the joint angle
double angle(Point p1, Point p2, Point p3)
{
	double dx1 = p1.x - p3.x;
	double dy1 = p1.y - p3.y;
	double dx2 = p2.x - p3.x;
	double dy2 = p2.y - p3.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

int main()
{
    std::cout << "Hello World!\n";

	//VideoCapture main("C:/Users/Lance/source/repos/IdentifyShapes/x64/Debug/shapes.mp4");
	VideoCapture main(2);

	int lH = 3, lS = 3, lV = 3, hH = 179, hS = 255, hV = 255;
	std::string trackbarWindowName = "Trackbars";
	namedWindow(trackbarWindowName);
	createTrackbar("Lower Hue", trackbarWindowName, &lH, 179);
	createTrackbar("Lower Saturation", trackbarWindowName, &lS, 255);
	createTrackbar("Lower Value", trackbarWindowName, &lV, 255);
	createTrackbar("Higher Hue", trackbarWindowName, &hH, 179);
	createTrackbar("Higher Saturation", trackbarWindowName, &hS, 255);
	createTrackbar("Higher Value", trackbarWindowName, &hV, 255);

	while (true)
	{
		if (waitKey(1) == 27)
		{
			break;
		}

		if (main.get(CAP_PROP_POS_FRAMES) == main.get(CAP_PROP_FRAME_COUNT))
		{
			main.set(CAP_PROP_POS_FRAMES, 0);
		}
		
		Mat img;
		main.read(img);

		Mat converted;
		cvtColor(img, converted, COLOR_BGR2HSV);
		imshow("hsv", converted);

		Mat thresholded;
		inRange(converted, Scalar(lH, lS, lV), Scalar(hH, hS, hV), thresholded);

		erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
		dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

		imshow("thresholded", thresholded);

		std::vector<std::vector<Point>> contours;
		std::vector<Point> approx;
		std::vector<Vec4i> hier;
		findContours(thresholded, contours, hier, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		Mat contoursMat;
		//drawContours(contoursMat, contours, -1, Scalar(0, 255, 0), 3, LINE_AA, hier, 3);
		//imshow("Contours", contoursMat);

		std::vector<std::vector<Point>> triangles;
		std::vector<std::vector<Point>> rectangles;
		std::vector<std::vector<Point>> hexagons;
		
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], approx, arcLength(contours[i], true) * .02, true);

			if (fabs(contourArea(approx)) > 1000 && isContourConvex(approx))
			{
				double maxCosine = 0, maxAngle = 0;
				switch (approx.size())
				{
				case 3:
					for (int i = 2; i < 4; i++)
					{
						double cosine = fabs(angle(approx[i % 3], approx[i - 2], approx[i - 1]));
						maxCosine = MAX(cosine, maxCosine);
					}

					maxAngle = acos(maxCosine) * (180 / 3.1415);

					if (maxAngle < 70)
					{
						triangles.push_back(approx);
					}
					break;
				case 4:
					for (int i = 2; i < 5; i++)
					{
						double cosine = fabs(angle(approx[i % 4], approx[i - 2], approx[i - 1]));
						maxCosine = MAX(cosine, maxCosine);
					}

					maxAngle = acos(maxCosine) * (180 / 3.1415);

					if (maxAngle < 100)
					{
						rectangles.push_back(approx);
					}
					break;
				case 6:
					for (int i = 2; i < 7; i++)
					{
						double cosine = fabs(angle(approx[i % 5], approx[i - 2], approx[i - 1]));
						maxCosine = MAX(cosine, maxCosine);
					}

					maxAngle = acos(maxCosine) * (180 / 3.1415);

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

		polylines(img, triangles, true, Scalar(255, 0, 0), 2, LINE_AA);
		polylines(img, rectangles, true, Scalar(0, 255, 0), 2, LINE_AA);
		polylines(img, hexagons, true, Scalar(0, 0, 255), 2, LINE_AA);
		

		for (std::vector<Point> s : triangles)
		{
			Rect rect = boundingRect(s);
			rectangle(img, rect, Scalar(255, 0, 0), 2, LINE_AA);
			putText(img, "triangle", rect.tl() - Point(0, 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 1, LINE_AA);
		}

		for (std::vector<Point> s : rectangles)
		{
			Rect rect = boundingRect(s);
			rectangle(img, rect, Scalar(255, 0, 0), 2, LINE_AA);
			putText(img, "rectangles", rect.tl() - Point(0, 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 1, LINE_AA);
		}

		for (std::vector<Point> s : hexagons)
		{
			Rect rect = boundingRect(s);
			rectangle(img, rect, Scalar(255, 0, 0), 2, LINE_AA);
			putText(img, "hexagons", rect.tl() - Point(0, 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, LINE_AA);
		}
		imshow("video", img);
	}
}
