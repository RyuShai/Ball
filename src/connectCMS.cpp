#include <ctime>
#include <queue>
#include <iostream>
#include <fstream>
#include <ctime>
#include <thread>
#include <chrono>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <curl/curl.h>
#include "base64.cpp"
#include "connectCMS.h"

#define UT_FUNCTION
#define URL_DEFAULT "http://demo.stmc.vp9.vn:8000/plateInfo"

using namespace std;
using namespace cv;
using namespace pr;

void postDataToCms(
    string camera_id, string typeViolation, string time,
    string location, string typeVehicle, string plate,
    string previousFrame, string firstRedLightFrame, string violationVehicleFrame)
{
    pr::ObjectUtils objects;
    CURLcode ret;
    CURL *hnd;
    struct curl_slist *slist1;

    std::string jsonstr = " {\"camera_id\":\" ";
    jsonstr.append(camera_id);

    jsonstr.append("\",\"typeViolation\":\" ");
    jsonstr.append(typeViolation);

    jsonstr.append("\",\"time\":\" ");
    jsonstr.append(time);

    jsonstr.append("\",\"location\":\" ");
    jsonstr.append(location);

    jsonstr.append("\",\"typeVehicle\":\" ");
    jsonstr.append(typeVehicle);

    jsonstr.append("\",\"plate\":\" ");
    jsonstr.append(plate);

    jsonstr.append("\",\"previousFrame\":\" ");
    jsonstr.append(previousFrame);

    jsonstr.append("\",\"firstRedLightFrame\":\" ");
    jsonstr.append(firstRedLightFrame);

    jsonstr.append("\",\"violationVehicleFrame\":\" ");
    jsonstr.append(violationVehicleFrame);

    jsonstr.append(" \" }");

    slist1 = NULL;
    slist1 = curl_slist_append(slist1, "Content-Type: application/json");
    string sever_address = "http://stmc.vp9.vn:8000/violation";
    hnd = curl_easy_init();
    char url[1000];
    strcpy(url, sever_address.c_str());
    // get plate
    curl_easy_setopt(hnd, CURLOPT_URL, url);
    curl_easy_setopt(hnd, CURLOPT_NOPROGRESS, 1L);
    curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, jsonstr.c_str());
    curl_easy_setopt(hnd, CURLOPT_USERAGENT, "VP9-ANPR");
    curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist1);
    curl_easy_setopt(hnd, CURLOPT_MAXREDIRS, 50L);
    curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(hnd, CURLOPT_TCP_KEEPALIVE, 1L);

    ret = curl_easy_perform(hnd);

    curl_easy_cleanup(hnd);
    hnd = NULL;
    curl_slist_free_all(slist1);
    slist1 = NULL;
    cout << endl;
    //delete hnd;
}

void push_data_to_CMS(
    string camera_id, string typeViolation, string time,
    string location, string typeVehicle, string plate,
    Mat previousFrame, Mat firstRedLightFrame, Mat violationVehicleFrame)
{
    string previousFrameString;
    vector<uchar> buf1;
    imencode(".jpg", previousFrame, buf1);
    uchar *enc_msg1 = new uchar[buf1.size()];
    for (int i = 0; i < (int)buf1.size(); i++)
        enc_msg1[i] = buf1[i];
    previousFrameString = base64_encode(enc_msg1, buf1.size());
    delete enc_msg1;

    string firstRedLightFrameString;
    vector<uchar> buf2;
    imencode(".jpg", firstRedLightFrame, buf2);
    uchar *enc_msg2 = new uchar[buf2.size()];
    for (int i = 0; i < (int)buf2.size(); i++)
        enc_msg2[i] = buf2[i];
    firstRedLightFrameString = base64_encode(enc_msg2, buf2.size());
    delete enc_msg2;

    string violationVehicleFrameString;
    vector<uchar> buf3;
    imencode(".jpg", violationVehicleFrame, buf3);
    uchar *enc_msg3 = new uchar[buf3.size()];
    for (int i = 0; i < (int)buf3.size(); i++)
        enc_msg3[i] = buf3[i];
    violationVehicleFrameString = base64_encode(enc_msg3, buf3.size());
    delete enc_msg3;

    postDataToCms(camera_id, typeViolation, time, location, typeVehicle, plate, previousFrameString, firstRedLightFrameString, violationVehicleFrameString);
}
