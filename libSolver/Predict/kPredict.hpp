#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Dense>
using namespace std;
using namespace cv;

/*
 *  kalmanFilter
 */
class kalmanFilter
{
public:

    // kalmanFilter();
    // ~kalmanFilter();

    void Init(double s, double v, int Id);

    void predict(int Id);

    double update(double m_s,double m_v,int Id);

    double getPredict(double x,int Id);

    //double x_speed=50;//m/s

private:

    Eigen::Matrix<double,2,1> x_;
    Eigen::Matrix<double,2,1> y_;
    Eigen::Matrix<double,2,1> z_;
    Eigen::Matrix<double,2,2> F1;
    Eigen::Matrix<double,2,2> P1;
    Eigen::Matrix<double,2,2> Q1;
    Eigen::Matrix<double,2,2> H1;
    Eigen::Matrix<double,2,2> K1;
    Eigen::Matrix<double,2,2> F2;
    Eigen::Matrix<double,2,2> P2;
    Eigen::Matrix<double,2,2> Q2;
    Eigen::Matrix<double,2,2> H2;
    Eigen::Matrix<double,2,2> K2;
    Eigen::Matrix<double,2,2> F3;
    Eigen::Matrix<double,2,2> P3;
    Eigen::Matrix<double,2,2> Q3;
    Eigen::Matrix<double,2,2> H3;
    Eigen::Matrix<double,2,2> K3;

};