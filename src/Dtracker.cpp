#include "Dtracker.hpp"

SingleTracker::SingleTracker(dlib::array2d<unsigned char> &_img, dlib::rectangle _rect, int global_id) {
//        this->setTrackID(std::rand() % 256);
    this->setTrackID(global_id+1);
    //cout << "Start tracking!!" << this->getTrackID() << endl;
    this->setRect(_rect);
    this->setCenter(_rect);
    tracker.start_track(_img, this->getRect());
    trackName = "";
    trackName.clear();
    lost_track_frame_count = 0;
    lost_track_frame_count_threshold = 3;
    trackQuality_threshold  = 7.0;
}

void SingleTracker::doSingleTrack(dlib::array2d<unsigned char>& _img) {
    // cout << "dosingleTrack" << endl;
    this->setTrackQuality(this->tracker.update(_img));
    dlib::rectangle pos = this->tracker.get_position();
    this->setRect(pos);
    this->setCenter(pos);
}

void SingleTracker::doSingleTrack_with_New_Bounding_Box(dlib::array2d<unsigned char>& _img, dlib::rectangle new_bounding_box) {    
    this->setRect(new_bounding_box);
    this->setCenter(new_bounding_box);
    this->tracker.start_track(_img, this->getRect());
}

bool SingleTracker::isNeedToDeleted() {
    bool yes_delete = false;
    //not delete track if its quality is still good
    if (this->getTrackQuality() > trackQuality_threshold) {
        return false;
    }else{ //when its quality is bad -> only delete track after some time > lost_track_frame_count_threshold (#frames)
        this->lost_track_frame_count++;
        if (this->lost_track_frame_count > lost_track_frame_count_threshold){
            yes_delete = true;
        }
    }

    return yes_delete;
}

bool SingleTracker::isMatchWithNewTrack(rectangle _rect) {
    long x_bar = _rect.left() + _rect.width() * 0.5;
    long y_bar = _rect.top() + _rect.height() * 0.5;
    drectangle pos = this->getRect();
    dlib::point center = this->getCenter();

    if ((pos.left() <= x_bar) && (x_bar <= pos.right()) && (pos.top() <= y_bar) &&
            (y_bar <= pos.bottom()) && (_rect.left() <= center.x()) &&
            (center.x() <= _rect.right()) && (_rect.top() <= center.y()) &&
            (center.y() <= _rect.bottom())) {
        return true;
    } else
        return false;
}

DtrackerManager::DtrackerManager(){
    this->global_trackid = 0;
}

int DtrackerManager::findMatchedTracker(rectangle _rect) {
    int res = -1;
    if (this->getTrackerList().size() > 0) {
        for (size_t i = 0; i < this->getTrackerList().size(); ++i) {
            if (this->getTrackerList()[i].isMatchWithNewTrack(_rect))
                res = this->getTrackerList()[i].getTrackID();
        }
    }
    return res;
}

void DtrackerManager::insertTracker(dlib::array2d<unsigned char>& _img,
                                    dlib::rectangle _rect, int find_matched_track_id) {

    //int find_matched_track_id = this->findMatchedTracker(_rect);

    if (find_matched_track_id == -1) {//if not found any available matched tracks, then create new track here
        SingleTracker newTracker(_img, _rect, this->global_trackid);
        this->global_trackid++;
        this->trackList.push_back(newTracker);
    }else{ //if found one available matched track, then update the track with new bounding box
        for (size_t i = 0; i < this->getTrackerList().size(); ++i) {
            if (this->getTrackerList()[i].getTrackID() == find_matched_track_id )
                this->getTrackerList()[i].doSingleTrack_with_New_Bounding_Box(_img, _rect);
        }
    }
}

int DtrackerManager::findTracker(int _trackId) {
    auto target = std::find_if(this->trackList.begin(), this->trackList.end(),
                               [&, _trackId](SingleTracker ptr) -> bool {
        return ptr.getTrackID() == _trackId;
    });

    if (target == this->trackList.end())
        return -1;
    else
        return target - this->trackList.begin();
}

void DtrackerManager::deleteTracker(int _trackId) {
    //cout << "delete track " << _trackId << endl;
    int result_idx = this->findTracker(_trackId);

    if (result_idx != -1) {
        // Remove SingleTracker object from the vector
        this->trackList.erase(trackList.begin() + result_idx);
    }
}
