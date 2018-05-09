#include "iostream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <ctime>
#include <thread>
#include <chrono>
#include <cstring>
#include <string>
#include <limits>
#include <sys/types.h>
#include <sys/stat.h>
#include <exception>
#include <math.h>
#include <string.h>
#include <time.h>
#include <algorithm>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <complex>

#include "ObjectUtils.h"
// #include "PlateRecognizator.h"
// #include "connectCMS.h"

#define DPRINTC(C) printf(#C " = %c\n", (C))
#define DPRINTS(S) printf(#S " = %s\n", (S))
#define DPRINTD(D) printf(#D " = %d\n", (D))
#define DPRINTLLD(LLD) printf(#LLD " = %lld\n", (LLD))
#define DPRINTLF(LF) printf(#LF " = %.5lf\n", (LF))

typedef long long lld;
typedef unsigned long long llu;
using namespace std;
using namespace cv;
using namespace pr;
int match_score, mismatch_score, gap_score;

int frameNum = 0;
int plate_num = 0;
int dp[50][50];

std::ofstream ofs;
std::string root_folder = "/home/sonhh/Desktop/Result";
string plate_folder_result = "/plate/";
string imageVehicle_folder_result = "/vehicle/";

const int Range_Time = 5;
const int MaxMissPlateOfQueue = 15;
const float distance_one_pixcel = 0.00001163; //10m / 480pixcel (tinh theo km)
const int type_plate_vehicle = 5;

//vehicle array_result[1001];
//bool n_array_result[1001];

// queue<FrameData> frameQueue1;

vector<Point2f> transformOffset(vector<Point2f> points, float x, float y)
{
	for (int i = 0; i < points.size(); i++)
	{
		points[i] = points[i] + Point2f(x, y);
	}
	return points;
}

void ObjectUtils::wtfile_demo(string link, string content)
{
	ofstream fout;
	fout.open(link, ios::app);
	assert(!fout.fail());
	fout << content << endl;
	fout.close();
	assert(!fout.fail());
}

string ObjectUtils::FloatToStr(float tt)
{
	stringstream ss; //convert tt to string
	ss << tt;
	string str = ss.str();
	return str;
}
std::string ObjectUtils::getCurrentDate()
{
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%Y_%m_%d", timeinfo);
	std::string str(buffer);
	return str;
}

long ObjectUtils::convert_time(string s)
{
	long hh, mm, ss;
	hh = (int)(s[10] - '0') * 10 + (int)(s[11] - '0');
	mm = (int)(s[13] - '0') * 10 + (int)(s[14] - '0');
	ss = (int)(s[16] - '0') * 10 + (int)(s[17] - '0');
	return hh * 3600 + mm * 60 + ss;
}
//===========================find best result=======================
int ObjectUtils::minResult(int x, int y, int z)
{
	int tmp = min(x, y);
	return min(tmp, z);
}

inline int needleman_wunsch(string A, string B, int length_str1, int length_str2)
{
	int n = length_str1;
	int m = length_str2;

	for (int i = 0; i <= n; i++)
		dp[i][0] = dp[0][i] = -i * gap_score;
	for (int i = 1; i <= n; i++)
	{
		for (int j = 1; j <= m; j++)
		{
			int S = (A[i - 1] == B[j - 1]) ? match_score : -mismatch_score;
			dp[i][j] = max(dp[i - 1][j - 1] + S, max(dp[i - 1][j] - gap_score, dp[i][j - 1] - gap_score));
		}
	}
	return dp[n][m];
}

inline pair<string, string> get_optimal_alignment(string A, string B, int length_str1, int length_str2)
{
	string retA, retB;
	stack<char> SA, SB;
	int ii = length_str1, jj = length_str2;
	while (ii != 0 || jj != 0)
	{
		if (ii == 0)
		{
			SA.push('-');
			SB.push(B[jj - 1]);
			jj--;
		}
		else if (jj == 0)
		{
			SA.push(A[ii - 1]);
			SB.push('-');
			ii--;
		}
		else
		{
			int S = (A[ii - 1] == B[jj - 1]) ? match_score : -mismatch_score;
			if (dp[ii][jj] == dp[ii - 1][jj - 1] + S)
			{
				SA.push(A[ii - 1]);
				SB.push(B[jj - 1]);
				ii--;
				jj--;
			}
			else if (dp[ii - 1][jj] > dp[ii][jj - 1])
			{
				SA.push(A[ii - 1]);
				SB.push('-');
				ii--;
			}
			else
			{
				SA.push('-');
				SB.push(B[jj - 1]);
				jj--;
			}
		}
	}
	while (!SA.empty())
	{
		retA += SA.top();
		retB += SB.top();
		SA.pop();
		SB.pop();
	}
	return make_pair(retA, retB);
}
int ObjectUtils::editDist(string str1, string str2, int m, int n)
{
	int dp1[m + 1][n + 1];
	for (int i = 0; i <= m; i++)
	{
		for (int j = 0; j <= n; j++)
		{
			if (i == 0)
				dp1[i][j] = j; // Min. operations = j
			else if (j == 0)
				dp1[i][j] = i; // Min. operations = i
			else if (str1[i - 1] == str2[j - 1])
				dp1[i][j] = dp1[i - 1][j - 1];
			else
				dp1[i][j] = 1 + minResult(dp1[i][j - 1],	  // Insert
										  dp1[i - 1][j],	  // Remove
										  dp1[i - 1][j - 1]); // Replace
		}
	}
	return dp1[m][n];
}

float ObjectUtils::calcBlurriness(const Mat &src)
{
	Mat Gx, Gy;
	Sobel(src, Gx, CV_32F, 1, 0);
	Sobel(src, Gy, CV_32F, 0, 1);
	double normGx = norm(Gx);
	double normGy = norm(Gy);
	double sumSq = normGx * normGx + normGy * normGy;
	return static_cast<float>(1000000. / (sumSq / src.size().area() + 1e-6));
}

bool init_array_result = true;

std::string ObjectUtils::getCurrentDateTime()
{
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%d-%m-%Y_%I-%M-%S", timeinfo);
	std::string str(buffer);
	return str;
}

std::string ObjectUtils::getCurrentDateTime_pushSQL()
{
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
	std::string str(buffer);
	return str;
}

string ObjectUtils::kindcascade(int mode)
{
	mode = 4;
	string kindcascade = "car_cascade:";
	if (mode == 0)
		kindcascade = "car_cascade:";
	if (mode == 1)
		kindcascade = "car_horizontal_cascade:";
	if (mode == 2)
		kindcascade = "car_redblue_cascade:";
	if (mode == 3)
		kindcascade = "car_redblue_horizontal_cascade:";
	if (mode == 4)
		kindcascade = "vehicle_cascade:";
	return kindcascade;
}

int ObjectUtils::kindsizecascade(int mode)
{
	int kindsizecascade = 0;
	if (mode == 0)
		kindsizecascade = 0;
	if (mode == 1)
		kindsizecascade = 1;
	if (mode == 2)
		kindsizecascade = 0;
	if (mode == 3)
		kindsizecascade = 1;
	return kindsizecascade;
}

struct line_equation // line equation ax+by+c=0
{
    float a;
    float b;
    float c;
};

//----------------Get line equation that cross two point p1 p2-----------------
line_equation get_line_equation(Point p1, Point p2)
{
    line_equation tmp;
    tmp.a = p1.y - p2.y;
    tmp.b = p2.x - p1.x;
    tmp.c = p1.x * p2.y - p2.x * p1.y;
    return (tmp);
}

//---------------Get line equation that cross 1 point and parallel to another line
line_equation get_line_equation2(Point p1, line_equation l1)
{
    line_equation tmp;
    tmp.a = l1.a;
    tmp.b = l1.b;
    tmp.c = (-1) * (l1.a * p1.x + l1.b * p1.y);
    return (tmp);
}
//--------------Get cross point of two lines in general form-----------------
Point get_cross_point(line_equation l1, line_equation l2)
{
    Point tmp;
    float denominator = l1.b * l2.a - l1.a * l2.b;
    if (denominator == 0)
    {
        tmp.x = 99999;
        tmp.y = 99999;
    }
    else
    {
        tmp.x = (int)(l1.c * l2.b - l1.b * l2.c) / denominator;
        tmp.y = (int)(l1.a * l2.c - l1.c * l2.a) / denominator;
    }
    return (tmp);
}
//---------------Get center point of two point---------------------
Point get_center_point(Point p1, Point p2)
{
    Point tmp;
    tmp.x = (int)(p1.x + p2.x) / 2;
    tmp.y = (int)(p1.y + p2.y) / 2;
    return (tmp);
}

//-----------------Get cross point----------------------------------
//                          -pB
//                   p1--------p2
//                    - -      -
//                    -        -
//                  --p3-------p4
//                -pA
//-------------------------------------------------------------------
Point ObjectUtils::get_cross_points(Point pA, Point pB, Rect vehicle_rect, int direction)
{
    Point tmp;
    line_equation road_line;
    Point p1, p2, p3, p4;
    p1.x = vehicle_rect.x;
    p1.y = vehicle_rect.y;
    p2.x = vehicle_rect.x + vehicle_rect.width;
    p2.y = vehicle_rect.y + vehicle_rect.y;
    p3.x = vehicle_rect.x;
    p3.y = vehicle_rect.y + vehicle_rect.height;
    p4.x = vehicle_rect.x + vehicle_rect.width;
    p4.y = vehicle_rect.y + vehicle_rect.height;
    //cout<<p1<<"\t"<<p2<<"\t"<<p3<<"\t"<<p4<<endl;
    road_line = get_line_equation(pA, pB);
    line_equation ref_line_1; // line that cross p3 and parallel to pA,pB
    ref_line_1 = get_line_equation2(p3, road_line);
    line_equation ref_line_2; // line that cross p1 and p4
    ref_line_2 = get_line_equation(p1, p4);
    if (direction == 1) // vehicle move from top to bottom
    {
        Point ref_point; // cross point of ref_line_1 and ref_line_2
        ref_point = get_cross_point(ref_line_1, ref_line_2);
        if ((ref_point.x == 99999) && (ref_point.y = 99999))
        {
            tmp = ref_point;
        }
        else
        {
            tmp = get_center_point(ref_point, p4);
        }
    }
    else // vehicle move from bottom to top
    {
        tmp = get_cross_point(ref_line_1, ref_line_2);
        //cout<<ref_liendl;
    }

    return (tmp);
}

float ObjectUtils::up_down(Point p, Point pA, Point pB)
{
    float tmp;//tmp >0, p is above line AB, tmp<0 p is under line AB, tmp=0, p is on line AB
    line_equation lineAB;
    lineAB=get_line_equation(pA,pB);
    tmp=(float) (lineAB.a*p.x+lineAB.b*p.y+lineAB.c);
    return (tmp);

}