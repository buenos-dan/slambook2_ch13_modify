#include "myslam/OrbMatcher.h"

namespace myslam {

void OrbMatcher::match(const Mat &descs1, const Mat &descs2, vector<DMatch> &matches) {
    for(int i = 0; i < descs1.rows; i++) {
        cv::DMatch match;
        int minDist = 255, minDist2 = 255;
        int matchIdx = -1;
        for(int j = 0; j < descs2.rows; j++) {
            int dist = DescriptorDistance(descs1.row(i), descs2.row(j));
            if(dist < minDist) {
                matchIdx = j;
                minDist2 = minDist;
                minDist = dist;
            }
            else if(dist < minDist2) {
                minDist2 = dist;
            }
        }
        match.queryIdx = i;
        match.trainIdx = matchIdx;
        match.distance = minDist;

        if(minDist < minDist2 * 0.7)
            matches.push_back(match);
    }
}

int OrbMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

}