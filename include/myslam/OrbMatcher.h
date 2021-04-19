#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace myslam {

class OrbMatcher {
public:
void match(const Mat &descs1, const Mat &descs2, vector<DMatch> &matches, float thRatio = 0.7);

private:
int DescriptorDistance(const Mat &a, const Mat &b);

};

}