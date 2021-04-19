#pragma once
#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

// #include "visual_odometry.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/g2o_types.h"
#include "myslam/feature.h"
#include "myslam/algorithm.h"
#include "DBoW2/DBoW2.h"

namespace myslam {

class LoopClosing{
public:
    typedef std::shared_ptr<LoopClosing> Ptr;
    // pass

public:
    LoopClosing();
    void Run();
    bool CheckNewKeyFrames();
    bool DetectLoop();
    void GlobalBA();
    void SetMap(Map::Ptr map);
    void SetVocab(BriefVocabulary * vocab);
    void SetFlag(std::atomic<bool> * flag);
    void SetLoopKFQueue(std::list<Frame::Ptr> * q);
    void SetMutexLoopQueeu(std::mutex *  m);
    void RunGlobalBA();

protected:
    std::list<Frame::Ptr> * loopKeyFrameQueue;
    std::mutex * mutexLoopQueue;
    Map::Ptr map_ = nullptr;
    // DBoW3::Vocabulary * vocab_ = nullptr;
    long lastLoopKFid = 0;
    double LOOPTHRES = 0.03;
    long startKFid = -1;

    BriefDatabase db_;
	BriefVocabulary* voc_;

    // ...
    std::thread loopclosing_thread_;

    std::atomic<bool> * globalBA_flag_;

    cv::Ptr<cv::DescriptorExtractor> descriptor_ = cv::ORB::create();


};
}

#endif  // MYSLAM_LOOPCLOSING_H