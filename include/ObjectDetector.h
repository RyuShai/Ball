#include <string.h>
#include <stdio.h>
#include <opencv2/video.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>

using namespace cv;

namespace pr
{
class ObjectDetector
{
  public:
	Mat GetObjectRegion(Mat img, bool isRedLight);
  public:
	int inputData;
};
}