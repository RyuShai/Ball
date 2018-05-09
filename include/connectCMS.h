#ifndef CONNECT_CMS
#define CONNECT_CMS

#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>
#include "ObjectUtils.h"

using namespace cv;
using namespace std;

void push_data_to_CMS(
    string camera_id, string typeViolation, string time,
    string location, string typeVehicle, string plate,
    Mat previousFrame, Mat firstRedLightFrame, Mat violationVehicleFrame);

#endif