/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/frame.h"

namespace myslam {

Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

int Frame::ExtractAndMatch(OrbExtractor *orbExtractor, OrbMatcher *orbMatcher) {
    vector<cv::KeyPoint> kps_left, kps_right;
    cv::Mat descs_left_raw, descs_right_raw;

    (*orbExtractor)(left_img_, kps_left, descs_left_raw);
    (*orbExtractor)(right_img_, kps_right, descs_right_raw);

    vector<cv::DMatch> matches;
    orbMatcher->match(descs_left_raw, descs_right_raw, matches);
    descs_left_.create(matches.size(), 32, CV_8UC1);
    for(int i = 0; i < matches.size(); i++) {
        Feature::Ptr feature_left(new Feature(shared_from_this(), kps_left[matches[i].trainIdx]));
        Feature::Ptr feature_right(new Feature(shared_from_this(), kps_right[matches[i].queryIdx]));
        feature_left->is_on_left_image_ = true;
        feature_right->is_on_left_image_ = false;
        features_left_.push_back(feature_left);
        features_right_.push_back(feature_right);

        descs_left_raw.row(matches[i].queryIdx).copyTo(descs_left_.row(i));
    }

    return matches.size();
}

cv::Mat Frame::GetDescriptors() {
    return descs_left_;
}



}
