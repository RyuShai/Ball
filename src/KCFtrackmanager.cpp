#include "KCFtrackmanager.hpp"

KCF_SingleTracker::KCF_SingleTracker(cv::Mat &_img, cv::Rect _rect, int global_id) {
//        this->setTrackID(std::rand() % 256);
    this->setTrackID(global_id+1);
    //cout << "Start tracking!!" << this->getTrackID() << endl;
    tracker.init(_img, _rect);
    this->setRect(_rect);
    this->setCenter(_rect);
    trackName = "";
    trackName.clear();
    lost_track_frame_count = 0;
    lost_track_frame_count_threshold = 3;
    //trackQuality_threshold  = 10.0;
}

void KCF_SingleTracker::doSingleTrack(cv::Mat& _img) {
    // cout << "dosingleTrack" << endl;
    this->setTrackQuality(tracker.track(_img));

    BBox_c bb = tracker.getBBox();
    cv::Rect pos(bb.cx - bb.w/2., bb.cy - bb.h/2., bb.w, bb.h);
    this->setRect(pos);
    this->setCenter(pos);
}

void KCF_SingleTracker::doSingleTrack_with_New_Bounding_Box(cv::Mat& _img, cv::Rect new_bounding_box) {
    this->setRect(new_bounding_box);
    this->setCenter(new_bounding_box);
    tracker.init(_img, new_bounding_box);
}

bool KCF_SingleTracker::isNeedToDeleted() {
    bool yes_delete = false;
    //not delete track if its quality is still good
    if (this->getTrackQuality() > trackQuality_threshold) {
        return false;
    }else{ //when its quality is bad -> only delete track after some time > lost_track_frame_count_threshold (#frames)
//        this->lost_track_frame_count++;
//        if (this->lost_track_frame_count > lost_track_frame_count_threshold){
//            yes_delete = true;
//        }
        return true;
    }

    return yes_delete;
}

bool KCF_SingleTracker::isMatchWithNewTrack(cv::Rect _rect) {
    long x_bar = _rect.x + _rect.width * 0.5;
    long y_bar = _rect.y + _rect.height * 0.5;
    cv::Rect pos = this->getRect();
    cv::Point center = this->getCenter();

    if ((pos.x <= x_bar) && (x_bar <= pos.x+pos.width) && (pos.y <= y_bar) &&
            (y_bar <= pos.y+pos.height) && (_rect.x <= center.x) &&
            (center.x <= _rect.x+_rect.width) && (_rect.y <= center.y) &&
            (center.y <= _rect.y+_rect.height)) {
        return true;
    } else
        return false;
}

/*---------------------------------------------------------------------------------

Function : isTargetInsideFrame

Check the target is inside the frame
If the target is going out of the frame, need to SingleTracker stop that target.

---------------------------------------------------------------------------------*/
int KCF_SingleTracker::isTargetInsideFrame(int _frame_width, int _frame_height)
{
    int cur_x = this->getCenter().x;
    int cur_y = this->getCenter().y;

    bool is_x_inside = ((0 <= cur_x) && (cur_x < _frame_width));
    bool is_y_inside = ((0 <= cur_y) && (cur_y < _frame_height));

    if (is_x_inside && is_y_inside)
        return true;
    else
        return false;
}


/////////////////////////////////////////////////////////////////////////////////////
/// \brief KCF_trackerManager::KCF_trackerManager
/////////////////////////////////////////////////////////////////////////////////////

KCF_trackerManager::KCF_trackerManager(){
    this->global_trackid = 0;
}

int KCF_trackerManager::findMatchedTracker(cv::Rect _rect) {
    int res = -1;
    if (this->getTrackerList().size() > 0) {
        for (size_t i = 0; i < this->getTrackerList().size(); ++i) {
            if (this->getTrackerList()[i].isMatchWithNewTrack(_rect))
                res = this->getTrackerList()[i].getTrackID();
        }
    }
    return res;
}

void KCF_trackerManager::insertTracker(cv::Mat& _img,
                                    cv::Rect _rect, int find_matched_track_id) {

    //int find_matched_track_id = this->findMatchedTracker(_rect);

    if (find_matched_track_id == -1) {//if not found any available matched tracks, then create new track here
        KCF_SingleTracker newTracker(_img, _rect, this->global_trackid);
        this->global_trackid++;
        this->trackList.push_back(newTracker);
    }else{ //if found one available matched track, then update the track with new bounding box
        for (size_t i = 0; i < this->getTrackerList().size(); ++i) {
            if (this->getTrackerList()[i].getTrackID() == find_matched_track_id )
                this->getTrackerList()[i].doSingleTrack_with_New_Bounding_Box(_img, _rect);
        }
    }
}

int KCF_trackerManager::findTracker(int _trackId) {
    auto target = std::find_if(this->trackList.begin(), this->trackList.end(),
                               [&, _trackId](KCF_SingleTracker ptr) -> bool {
        return ptr.getTrackID() == _trackId;
    });

    if (target == this->trackList.end())
        return -1;
    else
        return target - this->trackList.begin();
}

void KCF_trackerManager::deleteTracker(int _trackId) {
    //cout << "delete track " << _trackId << endl;
    int result_idx = this->findTracker(_trackId);

    if (result_idx != -1) {
        // Remove SingleTracker object from the vector
        this->trackList.erase(trackList.begin() + result_idx);
    }
}
std::map<int, cv::Rect> KCF_trackerManager::TrackerEndtoEnd(std::vector<cv::Rect> dets, cv::Mat frame){
    cv::Mat input_gray = frame.clone();
    if (frame.channels() == 3){
        cv::cvtColor(frame, input_gray, CV_BGR2GRAY);
        input_gray.convertTo(input_gray, CV_32FC1);
    }else
        frame.convertTo(input_gray, CV_32FC1);

    //If provided some detections, then we update tracks with detections
    if (dets.size() > 0) {
        //std::cout<< "detected face NUM : " << dets.size() << endl;

        for(int k = 0; k < dets.size(); k++)
        {
            int find_matched_track_id = this->findMatchedTracker(dets[k]);
            this->insertTracker(input_gray, dets[k], find_matched_track_id); //insert new track or update current track with new det
        }
    } else{
        for (size_t i = 0; i < this->getTrackerList().size(); ++i) {
            this->getTrackerList()[i].doSingleTrack(input_gray);

            //Delete tracks if needed
            if (this->getTrackerList()[i].isTargetInsideFrame(frame.cols,frame.rows) ==false ||
                    this->getTrackerList()[i].isNeedToDeleted()) {
                this->deleteTracker(this->getTrackerList()[i].getTrackID());
            }
        }
    }

    std::map<int, cv::Rect> mapRes;
    for(int i = 0; i < this->getTrackerList().size(); i++)
    {
        mapRes.insert(std::make_pair(this->getTrackerList()[i].getTrackID(), this->getTrackerList()[i].getRect()));
    }

    return mapRes;
}
