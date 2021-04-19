//
// Created by buenos-dan on 21-4-17.
//

#include "myslam/loopclosing.h"

namespace myslam {

    void LoopClosing::Run()
    {
        while(1)
        {
            if(CheckNewKeyFrames())
            {
                if(DetectLoop())
                {
                    GlobalBA();
                }
            }       
            usleep(5000);
        }
    }

    bool LoopClosing::CheckNewKeyFrames()
    {
        std::unique_lock<std::mutex> lock(mutexLoopQueue);
        return(!loopKeyFrameQueue.empty());
    }

    bool LoopClosing::DetectLoop(){
        Frame::Ptr currentKF_ = loopKeyFrameQueue.front();
        loopKeyFrameQueue.pop_front();
        if(currentKF_ ->id_ < lastLoopKFid + 10){
            return false;
        }

        cv::Mat descriptor_ = currentKF_ -> GetDescriptors();
        Map::KeyframesType keyFrames_ = map_ -> GetAllKeyFrames();  // unordered_map<long, Frame>
        for (auto it = keyFrames_.begin(); it != keyFrames_.end(); ++it) {
            // DBoW3::BowVector v1, v2;
            // cv::Mat prevDescriptor = it -> second -> GetDescriptor();
            // vocab.transform(descriptor_, v1);
            // vocab.transform(prevDescriptor, v2);
            // for (int j = i; j < images.size(); j++) {
            //     DBoW3::BowVector v2;
            //     vocab.transform(descriptors[j], v2);
            //     double score = vocab.score(v1, v2);
            //     cout << "image " << i << " vs image " << j << " : " << score << endl;
            // }
            // cout << endl;
        }
        return true;
    }

    void LoopClosing::GlobalBA(){
        
    }

    void LoopClosing::SetMap(Map::Ptr map) { map_ = map; }
}