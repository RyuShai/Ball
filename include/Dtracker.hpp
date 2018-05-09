#ifndef DTRACKER_HPP
#define DTRACKER_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dlib/dnn.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>

using namespace dlib;
using namespace std;
class SingleTracker {
public:
    correlation_tracker tracker;
    int trackId;
    string trackName;
    std::vector<string> allFaceIDs;
    std::vector<dlib::matrix<dlib::rgb_pixel>> allFaceImages;
    dlib::rectangle rect;
    dlib::point center;
    double trackQuality;
    double trackQuality_threshold;
    int lost_track_frame_count; //The nummber of frames from the track is lost (track quality < threshold)
    int lost_track_frame_count_threshold;

public:

    SingleTracker(dlib::array2d<unsigned char> &_img, dlib::rectangle _rect, int global_id);

    int getTrackID() { return this->trackId; }

    string getTrackName() { return this->trackName; }

    dlib::rectangle getRect() { return this->rect; }

    dlib::point getCenter() { return this->center; }

    double getTrackQuality() { return this->trackQuality; }

    void setTrackID(int _trackId) { this->trackId = _trackId; }

    void setTrackName(string _trackName) { this->trackName = _trackName; }

    void setRect(dlib::drectangle _drect) { this->rect = _drect; }

    void setCenter(dlib::drectangle _drect) {
        this->center =
                dlib::point((long)(_drect.tl_corner().x() + (_drect.width() / 2)),
                            (long)(_drect.tl_corner().y() + (_drect.height() / 2)));
    }

    void setTrackQuality(double _trackQuality) {
        this->trackQuality = _trackQuality;
    }

    /* Core Function */
    void doSingleTrack(dlib::array2d<unsigned char> &_img);
    void doSingleTrack_with_New_Bounding_Box(dlib::array2d<unsigned char>& _img, dlib::rectangle new_bounding_box);

    bool isNeedToDeleted();

    bool isMatchWithNewTrack(dlib::rectangle _rect);
};

class DtrackerManager {
public:
    std::vector<SingleTracker> trackList;
    int global_trackid;
    int *frame_count;

public:
    DtrackerManager();
    ~DtrackerManager(){}
    /* Get Function */
    std::vector<SingleTracker> &getTrackerList() {
        return this->trackList;
    }  // Return reference! not value!

    /* Core Function */

    //return -1 if there are no tracks found matched
    int findMatchedTracker(dlib::rectangle _rect);

    void insertTracker(dlib::array2d<unsigned char> &_img, dlib::rectangle _rect, int find_matched_track_id);

    int findTracker(int _trackId);

    void deleteTracker(int _trackId);
};

#endif  // DTRACKER_HPP
