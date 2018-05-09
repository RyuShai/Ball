#include "RedLightDetector.h"
#include <vector>
#include <string.h>
#include <stdio.h>
#include "IOData.h"
#include <stdio.h>
#include <fstream>
#include <iostream>

using namespace pr;

#define Redcolor cv::Scalar(0, 0, 255)
#define Greencolor cv::Scalar(0, 255, 0)
#define Yellowcolor cv::Scalar(0, 255, 255)
#define Bluecolor cv::Scalar(255, 0, 0)
#define lightcyan cv::Scalar(255, 255, 22)

string dataConfig = "../data/Config.txt";

const cv::Point2f R_location(atoi(IOData::GetCongfigData(dataConfig, "xR:").c_str()), atoi(IOData::GetCongfigData(dataConfig, "yR:").c_str()));
const cv::Point2f Y_location(atoi(IOData::GetCongfigData(dataConfig, "xYe:").c_str()), atoi(IOData::GetCongfigData(dataConfig, "yYe:").c_str()));
const cv::Point2f G_location(atoi(IOData::GetCongfigData(dataConfig, "xG:").c_str()), atoi(IOData::GetCongfigData(dataConfig, "yG:").c_str()));


const double RADIUS = 15;

cv::Mat pr::RedLightDetector::DetectRedLight(Mat img)
{
	cv::Mat gray;
	cvtColor(img, gray, CV_BGR2GRAY);
	cv::Mat maskR = cv::Mat::zeros(gray.size(), CV_8UC1);
	maskR.at<uchar>(R_location) = 255;

	cv::Mat maskG = cv::Mat::zeros(gray.size(), CV_8UC1);
	maskG.at<uchar>(G_location) = 255;

	cv::Mat maskY = cv::Mat::zeros(gray.size(), CV_8UC1);
	maskY.at<uchar>(Y_location) = 255;

	int kernel_size = 3;
	cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
											cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1),
											cv::Point(kernel_size, kernel_size));

	dilate(maskR, maskR, element);
	dilate(maskG, maskG, element);
	dilate(maskY, maskY, element);
	cv::Mat light_R, light_G, light_Y;
	gray.copyTo(light_R, maskR);
	gray.copyTo(light_G, maskG);
	gray.copyTo(light_Y, maskY);

	int sumR = sum(light_R)[0];
	int sumG = sum(light_G)[0];
	int sumY = sum(light_Y)[0];

	if (sumR > sumG && sumR > sumY)
	{
		circle(img, R_location, RADIUS, cv::Scalar(0, 0, 255), 2, 8, 0);
		isRedLight = true;
		isGlight = false;
		isYlight = false;
		;
		putText(img, "Do", Y_location + cv::Point2f(-15, 0), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, Redcolor, 2, CV_AA);
	}

	if (sumY > sumG && sumY > sumR)
	{
		circle(img, Y_location, RADIUS, cv::Scalar(0, 0, 255), 2, 8, 0);
		isRedLight = false;
		isGlight = false;
		isYlight = true;
		cv::putText(img, "Vang", Y_location + cv::Point2f(-15, 0), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, Yellowcolor, 2, CV_AA);
	}

	if (sumG > sumR && sumG > sumY)
	{
		cv::circle(img, G_location, RADIUS, cv::Scalar(0, 0, 255), 2, 8, 0);
		isRedLight = false;
		isGlight = true;
		isYlight = false;
		cv::putText(img, "Xanh", Y_location + cv::Point2f(-15, 0), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, lightcyan, 2, CV_AA);
	}

	return img;
}
