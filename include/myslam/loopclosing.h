#pragma once
#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

// #include "visual_odometry.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/g2o_types.h"
#include "myslam/feature.h"
#include "myslam/algorithm.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

namespace myslam {

class LoopClosing{
public:
    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;
    typedef std::shared_ptr<LoopClosing> Ptr;
    // pass

public:
    LoopClosing();
    void Run();
    bool CheckNewKeyFrames();
    bool DetectLoop();
    void GlobalBA();
    void SetMap(Map::Ptr map);
    void SetVocab(ORBVocabulary * vocab);
    void SetFlag(std::atomic<bool> * flag);
    void RunGlobalBA();

protected:
    std::list<Frame::Ptr> loopKeyFrameQueue;
    std::mutex mutexLoopQueue;
    Map::Ptr map_ = nullptr;
    ORBVocabulary * vocab_ = nullptr;
    long lastLoopKFid = 0;
    double LOOPTHRES = 0.03;
    long startKFid = -1;

    // ...
    std::thread loopclosing_thread_;

    std::atomic<bool> * globalBA_flag_;


};
}

#endif  // MYSLAM_LOOPCLOSING_H