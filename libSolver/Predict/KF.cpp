#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "KF.hpp"
using namespace cv;
using namespace std;
 
const int winHeight=600;
const int winWidth=800;
 
 
Point2f kalmanFilter::KF (Point Position)
{
	RNG rng;
	//1.kalman filter setup
	const int stateNum=4;                                      //状态值4×1向量(x,y,△x,△y)
	const int measureNum=2;                                    //测量值2×1向量(x,y)	
	KalmanFilter KF(stateNum, measureNum, 0);	
 
	KF.transitionMatrix = (Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
	rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
	
	//namedWindow("kalman");
	//setMouseCallback("kalman",mouseEvent);
		
	//Mat image(winHeight,winWidth,CV_8UC3,Scalar(0));
 
	
	//2.kalman prediction
	Mat prediction = KF.predict();
	Point predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1));   //预测值(x',y')

	//3.update measurement
	measurement.at<float>(0) = (float)Position.x;
	measurement.at<float>(1) = (float)Position.y;		
 
	//4.update
	KF.correct(measurement);
	Point2f realKF;
	if(Position.x-predict_pt.x>0)
	{
		realKF=Point2f (predict_pt.x+(Position.x-predict_pt.x),predict_pt.y+(Position.y-predict_pt.y));
	}
	else{
		realKF=Point2f (predict_pt.x+(Position.x-predict_pt.x),predict_pt.y+(Position.y-predict_pt.y));
	}

	return realKF;
	//circle(img,predict_pt,5,Scalar(0,255,0),3);    //predicted point with green
	//circle(image,Position,5,Scalar(255,0,0),3); //current position with red		
	
}

vector<Point2f> kalmanFilter::setKF(Point Position1,Point Position2,Point Position3,Point Position4)
{
	Kpoints.resize(4);
	Kpoints[0] = Position1;
	Kpoints[1] = Position2;
	Kpoints[2] = Position3;
	Kpoints[3] = Position4;
	return Kpoints;
}