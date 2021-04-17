#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

namespace myslam {

class OrbExtractor {
public:

    OrbExtractor(int nLevels, float factor, int thFast, int nFeatures);

    void operator () (const Mat &img, vector<KeyPoint> &kps, Mat &descs);

private:

    Ptr<ORB> orbExtractor_;

};

}