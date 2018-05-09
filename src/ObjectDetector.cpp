#include "ObjectDetector.h"
#include <vector>
#include <string.h>
#include <stdio.h>
#include "IOData.h"

#include <dlib/dnn.h>
#include <dlib/opencv.h>
#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include "Dtracker.hpp"

using namespace pr;

cv::Mat pr::ObjectDetector::GetObjectRegion(Mat smallFrame, bool isRedLight)
{
	string stringConfig = "../data/Config.txt";
	typedef scan_fhog_pyramid<pyramid_down<6>> image_scanner_type;
	object_detector<image_scanner_type> detector, detector_car;
	deserialize("../data/model/moto_detector.svm") >> detector;
	// deserialize("../data/model/car_detector.svm") >> detector_car;
	std::vector<object_detector<image_scanner_type>> my_detectors;
	my_detectors.push_back(detector);
	//my_detectors.push_back(detector_car);
	double epsilon = 0.1;
	int count_frame = 0;
	DtrackerManager dtrackers;
	dtrackers.frame_count = &count_frame;
	bool isTracking = false;
	double scale = 2;
	string imgName, s;
	std::vector<cv::Point> pts_violation;
	int roi_left = atoi(IOData::GetCongfigData(stringConfig, "cropX:").c_str());
	int roi_top = atoi(IOData::GetCongfigData(stringConfig, "cropY:").c_str());
	int roi_width = atoi(IOData::GetCongfigData(stringConfig, "cropWidth:").c_str());
	int roi_height = atoi(IOData::GetCongfigData(stringConfig, "cropHeight:").c_str());
	cv::Rect rect_roi(roi_left, roi_top, roi_width, roi_height);
	cv::rectangle(smallFrame, rect_roi, cv::Scalar(255, 0, 255), 2);
	cv::Mat img_roi = smallFrame(rect_roi);

	cv_image<bgr_pixel> cimg_roi(img_roi);
	cv_image<bgr_pixel> cimg(smallFrame);

	dlib::array2d<unsigned char> img_gray;
	dlib::array2d<unsigned char> img_gray_roi;

	dlib::matrix<dlib::rgb_pixel> spimg;
	dlib::matrix<dlib::rgb_pixel> spimg_roi;
	dlib::assign_image(spimg, cimg);
	dlib::assign_image(img_gray, spimg);

	dlib::assign_image(spimg_roi, cimg_roi);
	dlib::assign_image(img_gray_roi, spimg_roi);

	if (dtrackers.getTrackerList().size() > 0)
	{
		for (size_t i = 0; i < dtrackers.getTrackerList().size(); ++i)
		{
			dtrackers.getTrackerList()[i].doSingleTrack(img_gray);

			if (dtrackers.getTrackerList()[i].isNeedToDeleted())
			{
				dtrackers.deleteTracker(dtrackers.getTrackerList()[i].getTrackID());
			}
		}
	}

	std::vector<dlib::rectangle> dets = evaluate_detectors(my_detectors, img_gray_roi);
	for (dlib::rectangle d : dets)
	{
		if (((long)d.area() > 150000) || ((long)d.area() < 20000))
			continue;
		dlib::rectangle dd(d.left() + roi_left, d.top() + roi_top, d.right() + roi_left, d.bottom() + roi_top);

		int find_matched_track_id = dtrackers.findMatchedTracker(dd);

		dtrackers.insertTracker(img_gray, dd, find_matched_track_id);
	}
	cout << "size: " << dtrackers.getTrackerList().size() << endl;
	for (int i = 0; i < dtrackers.getTrackerList().size(); i++)
	{
		int track_id = dtrackers.getTrackerList()[i].getTrackID();
		dlib::rectangle det = dtrackers.getTrackerList()[i].getRect();
		cv::rectangle(smallFrame, cv::Point(det.left(), det.top()), cv::Point(det.right(), det.bottom()), cv::Scalar(0, 255, 255), 2);
		cv::putText(smallFrame, std::to_string(track_id), cv::Point(det.left(), det.top()), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(255, 255, 255), 1, 8);
	}
	return smallFrame;
}
