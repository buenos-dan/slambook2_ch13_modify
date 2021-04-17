#pragma once
#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

#include "myslam/frame.h"
#include "myslam/map.h"
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

namespace myslam {

class LoopClosing{
public:
    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
        ORBVocabulary;

public:
    LoopClosing();
    void Run();
    bool CheckNewKeyFrames();
    bool DetectLoop();
    void GlobalBA();
    void SetMap(Map::Ptr map);

protected:
    std::list<Frame::Ptr> loopKeyFrameQueue;
    std::mutex mutexLoopQueue;
    Map::Ptr map_ = nullptr;
    long lastLoopKFid = 0;

};
}

#endif  // MYSLAM_LOOPCLOSING_H