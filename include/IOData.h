#ifndef I_O_DATA_H
#define I_O_DATA_H

#include <opencv2/opencv.hpp>
#include <iostream>

enum INPUT_MODE
{
	I_VIDEO,
	I_CAMERA
};

enum DETECT_MODE
{
	CAR_4,
	CAR_5,
	MOTOBIKE_4,
	MOTOBIKE_5
};
using namespace std;
namespace pr

{

class IOData
{
  public:
	static std::string GetLinkURL(string linkFileConfig);
	static INPUT_MODE GetInputMode(string linkFileConfig);
	static DETECT_MODE GetDetectMode(string linkFileConfig);
	static std::string GetCarPlateCascade(string linkFileConfig,int mode);
	static std::string GetMotobikePlateCascade(string linkFileConfig);
	static std::string GetCarPlatelogistic(string linkFileConfig);
	static std::string GetMotobikePlatelogistic(string linkFileConfig);
	static std::string GetCongfigData(string linkFileConfig,std::string key);
};
}

#endif