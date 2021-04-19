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
            LOG(INFO)<< "loopclosing is Running...";
            usleep(5000);
        }
    }

    bool LoopClosing::CheckNewKeyFrames()
    {
        std::unique_lock<std::mutex> lock(*mutexLoopQueue);
        return(!loopKeyFrameQueue -> empty());
    }

    bool LoopClosing::DetectLoop(){
        Frame::Ptr currentKF_;
        {
            std::unique_lock<std::mutex> lock(*mutexLoopQueue);
            currentKF_ = loopKeyFrameQueue -> front();
            loopKeyFrameQueue -> pop_front();
        }
       
        if(currentKF_ ->id_ < lastLoopKFid + 50){
            return false;
        }

        QueryResults ret;
        // TicToc t_query;
        db.query(currentKF_->briefDescriptors, ret, 4, frame_index - 50);
        //printf("query time: %f", t_query.toc());
        //cout << "Searching for Image " << frame_index << ". " << ret << endl;

        // TicToc t_add;
        db.add(currentKF_->briefDescriptors);

        // LOG(INFO) << "detectLoop ...";

        // cv::Mat descriptor = currentKF_ -> GetDescriptor();
        // if(descriptor.empty()){
        //     LOG(INFO) << "descriptor is nullptr ...";
        //     return false;
        // }
        // DBoW3::BowVector v1, v2;
        // // return false;
        // vocab_ -> transform(cv::Mat(), v1);
        // LOG(INFO) << "here";
        // Map::KeyframesType keyFrames_ = map_ -> GetAllKeyFrames();  // unordered_map<long, Frame>
        // long maxScoreKFid = -1;
        // double maxScore = 0;
        // for (auto it = keyFrames_.begin(); it != keyFrames_.end(); ++it) {
        //     cv::Mat prevDescriptor = it -> second -> GetDescriptor();
        //     vocab_ -> transform(prevDescriptor, v2);
              
        //     double score = vocab_ -> score(v1, v2);
        //     if(score > maxScore){
        //         score = maxScore;
        //         maxScoreKFid = it -> first;
        //     }
        // }

        // if(maxScore < LOOPTHRES){
        //     return false;
        // }

        // std::cout<< "check loopclosing, start GBA." << std::endl;
        // startKFid = maxScoreKFid;
        return true;
    }

    void LoopClosing::GlobalBA(){
        LOG(INFO) << "start gba";
        // std::thread gba_ = std::thread(std::bind(&LoopClosing::RunGlobalBA, this));
         // notify backend to run globalBA.
        globalBA_flag_->store(true);
    }

    void LoopClosing::RunGlobalBA(){
       
    }

    void LoopClosing::SetMap(Map::Ptr map) { map_ = map; }

    void LoopClosing::SetVocab(BriefVocabulary * vocab){ 
        voc_ = vocab; 
        db_.setVocabulary(*voc_, false, 0);
    };

    void LoopClosing::SetFlag(std::atomic<bool> * flag) { globalBA_flag_ = flag; }

    void LoopClosing::SetLoopKFQueue(std::list<Frame::Ptr> * q) { loopKeyFrameQueue = q; };
    void LoopClosing::SetMutexLoopQueeu(std::mutex *  m){ mutexLoopQueue = m; };
}