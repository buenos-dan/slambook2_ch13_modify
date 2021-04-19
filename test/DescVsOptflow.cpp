#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "myslam/OrbExtractor.h"
#include "myslam/OrbMatcher.h"

using namespace std;
using namespace cv;
using namespace myslam;

void stereoMatchOptflow(Mat imgLeft, Mat imgRight) {
    OrbExtractor orbExtractor(1, 1, 1, 1000, 1.2, 8, 20, 7);
    vector<KeyPoint> kpsLeft, kpsRight;
    Mat descsLeft;
    vector<Point2f> kpsPtLeft, kpsPtRight;
    vector<uchar> status;
    Mat error;
    // orbExtractor(imgLeft, kpsLeft, descsLeft);

    Ptr<GFTTDetector> gftt_ = cv::GFTTDetector::create(1000, 0.01, 20);
    gftt_->detect(imgLeft, kpsLeft);

    for(auto &kp : kpsLeft) {
        kpsPtLeft.push_back(kp.pt);
        kpsPtRight.push_back(kp.pt);
    }

    cv::calcOpticalFlowPyrLK(
        imgLeft, imgRight, kpsPtLeft, kpsPtRight,
        status, error, cv::Size(11, 11), 1);

    for(auto &kpPt : kpsPtRight) {
        KeyPoint curKp;
        curKp.pt = kpPt;
        kpsRight.push_back(curKp);
    }
    
    vector<DMatch> matches;
    for(int i = 0; i < status.size(); i++) {
        if(status[i]) {
            DMatch curMatch;
            curMatch.queryIdx = i;
            curMatch.trainIdx = i;
            matches.push_back(curMatch);
        }
    }
    printf("optflow: %d matches\n", matches.size());

    Mat imgOut;
    drawMatches(imgLeft, kpsLeft, imgRight, kpsRight, matches, imgOut);
    imshow("orb", imgOut);
    cvWaitKey(0);
}

void stereoMatchOrb(Mat imgLeft, Mat imgRgiht) {
    OrbExtractor orbExtractor(1, 1, 1, 1000, 1.2, 8, 20, 7);
    vector<KeyPoint> kpsLeft, kpsRight;
    Mat descsLeft, descsRight;
    orbExtractor(imgLeft, kpsLeft, descsLeft);
    orbExtractor(imgRgiht, kpsRight, descsRight);

    OrbMatcher orbMatcher;
    vector<DMatch> matches;
    orbMatcher.match(descsLeft, descsRight, matches, 0.9);
    printf("orb: %d matches\n", matches.size());

    Mat imgOut;
    drawMatches(imgLeft, kpsLeft, imgRgiht, kpsRight, matches, imgOut);
    imshow("orb", imgOut);
    cvWaitKey(0);
}



int main(int argc, char **argv) {
    Mat imgLeft, imgRight;
    if(argc != 2) {
        imgLeft = imread("/home/yuhaorong/slamDataset/kitti/00/image_0/000000.png", CV_LOAD_IMAGE_GRAYSCALE);
        imgRight = imread("/home/yuhaorong/slamDataset/kitti/00/image_1/000000.png", CV_LOAD_IMAGE_GRAYSCALE);
        printf("usage: OrbVsOptflow <img1> <img2>\n");
    } else {
        imgLeft = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
        imgRight = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
    }

    if(imgLeft.data == nullptr) {
        printf("failed to load imgLeft\n");
        return -1;
    }
    if(imgRight.data == nullptr) {
        printf("failed to load imgRight\n");
        return -1;
    }

    stereoMatchOptflow(imgLeft, imgRight);
    stereoMatchOrb(imgLeft, imgRight);
}
