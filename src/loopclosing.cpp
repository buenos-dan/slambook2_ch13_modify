//
// Created by buenos-dan on 21-4-17.
//

#include "myslam/loopclosing.h"

namespace myslam {

    LoopClosing::LoopClosing() {
        // backend_running_.store(true);
        loopclosing_thread_ = std::thread(std::bind(&LoopClosing::Run, this));
    }

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

        cv::Mat descriptor_ = GetDescriptor(currentKF_);
        Map::KeyframesType keyFrames_ = map_ -> GetAllKeyFrames();  // unordered_map<long, Frame>
        long maxScoreKFid = -1;
        double maxScore = 0;
        for (auto it = keyFrames_.begin(); it != keyFrames_.end(); ++it) {
            DBoW2::BowVector v1, v2;
            cv::Mat prevDescriptor = GetDescriptor(it -> second);
            vocab_ -> transform(descriptor_, v1);
            vocab_ -> transform(prevDescriptor, v2);
            double score = vocab_ -> score(v1, v2);
            if(score > maxScore){
                score = maxScore;
                maxScoreKFid = it -> first;
            }
        }

        if(maxScore < LOOPTHRES){
            return false;
        }

        std::cout<< "check loopclosing, start GBA." << std::endl;
        startKFid = maxScoreKFid;
        return true;
    }

    void LoopClosing::GlobalBA(){
        std::thread gba_ = std::thread(std::bind(&LoopClosing::RunGlobalBA, this));
    }

    void LoopClosing::RunGlobalBA(){
        // notify backend to run globalBA.
        globalBA_flag_->store(true);
    }

    void LoopClosing::SetMap(Map::Ptr map) { map_ = map; }

    void LoopClosing::SetVocab(ORBVocabulary * vocab) { vocab_ = vocab; }

    void LoopClosing::SetFlag(std::atomic<bool> * flag) { globalBA_flag_ = flag; }

    cv::Mat LoopClosing::GetDescriptor(Frame::Ptr frame){
        std::vector<std::shared_ptr<Feature>> features = frame -> features_left_;
        std::vector<cv::KeyPoint> kps;
        cv::Mat descptors;
        for(std::shared_ptr<Feature> feat: features){
            kps.push_back(feat -> position_);
        }
        descriptor_->compute(frame -> left_img_, kps, descptors);
        return descptors;
    }
}