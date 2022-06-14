#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
using namespace std;
using namespace cv;

class kalmanFilter
{
    public: 
    
    Point2f KF (Point Position);
    vector<Point2f> setKF(Point Position1,Point Position2,Point Position3,Point Position4);
    Point2f predictPoint;
    vector<Point2f> Kpoints;
    //private:

};