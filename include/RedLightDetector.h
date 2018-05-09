#include <string.h>
#include <stdio.h>
#include <opencv2/video.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>

using namespace cv;

namespace pr
{
class RedLightDetector
{
public:
	bool isRedLight = false;
	bool isGlight = false;
	bool isYlight = false;
	Mat DetectRedLight(Mat smallFrame);

public:
	int inputData;
};
}