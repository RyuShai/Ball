//#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include "kcf.hpp"

class KCF_SingleTracker {
public:
    KCF_Tracker tracker;
    int trackId;
    std::string trackName;
    std::vector<std::string> allFaceIDs;
    cv::Rect rect;
    cv::Point center;
    double trackQuality;
    double trackQuality_threshold = 10.;
    int lost_track_frame_count; //The nummber of frames from the track is lost (track quality < threshold)
    int lost_track_frame_count_threshold;

public:

    KCF_SingleTracker(cv::Mat &_img, cv::Rect _rect, int global_id);

    int getTrackID() { return this->trackId; }

    std::string getTrackName() { return this->trackName; }

    cv::Rect getRect() { return this->rect; }

    cv::Point getCenter() { return this->center; }

    double getTrackQuality() { return this->trackQuality; }

    void setTrackID(int _trackId) { this->trackId = _trackId; }

    void setTrackName(std::string _trackName) { this->trackName = _trackName; }
    void setRect(cv::Rect _drect) { this->rect = _drect; }

    void setCenter(cv::Rect _drect) {
        this->center =
                cv::Point((long)(_drect.x + (_drect.width / 2)),
                            (long)(_drect.y + (_drect.height / 2)));
    }
    void setTrackQuality(double _trackQuality) {
        this->trackQuality = _trackQuality;
    }

    /* Core Function */
    void doSingleTrack(cv::Mat &_img);
    void doSingleTrack_with_New_Bounding_Box(cv::Mat& _img, cv::Rect new_bounding_box);

    bool isNeedToDeleted();

    bool isMatchWithNewTrack(cv::Rect _rect);
    int isTargetInsideFrame(int _frame_width, int _frame_height);
};

class KCF_trackerManager {
public:
    std::vector<KCF_SingleTracker> trackList;
    int global_trackid;
    int *frame_count;

public:
    KCF_trackerManager();
    ~KCF_trackerManager(){}
    /* Get Function */
    std::vector<KCF_SingleTracker> &getTrackerList() {
        return this->trackList;
    }  // Return reference! not value!

    /* Core Function */

    //return -1 if there are no tracks found matched
    int findMatchedTracker(cv::Rect _rect);

    void insertTracker(cv::Mat &_img, cv::Rect _rect, int find_matched_track_id);

    int findTracker(int _trackId);

    void deleteTracker(int _trackId);

    std::map<int, cv::Rect> TrackerEndtoEnd(std::vector<cv::Rect> dets, cv::Mat frame);
};


