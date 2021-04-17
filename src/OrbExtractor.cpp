#include "myslam/OrbExtractor.h"

namespace myslam {

OrbExtractor::OrbExtractor(int nLevels, float factor, int thFast, int nFeatures) {
    orbExtractor_ = ORB::create(nFeatures, factor, nLevels, thFast);
}

void OrbExtractor::operator () (const Mat &img, vector<KeyPoint> &kps, Mat &descs) {
    orbExtractor_->detectAndCompute(img, Mat(), kps, descs);
}

}