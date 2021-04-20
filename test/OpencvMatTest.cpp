#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    const int nrows = 3, ncols = 5;
    Mat a[nrows], b;
    
    for(int i = 0; i < nrows; i++) {
        a[i].create(1, ncols, CV_8UC1);
        for(int j = 0; j < ncols; j++) {
            a[i].at<uchar>(0, j) = i;
        }
    }

    a[1].copyTo(b);

    Mat c;
    c.create(nrows, ncols, CV_8UC1);
    for(int i = 0; i < nrows; i++) {
        a[i].copyTo(c.row(i));
    }

    a[1].at<uchar>(2) = 3;
    c.at<uchar>(2) = 5;

    cout << "a[1]:\n" << a[1] << endl
         << "b:\n" << b << endl
         << "c:\n" << c << endl;

    return 0;
}
