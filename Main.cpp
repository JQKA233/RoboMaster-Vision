#include<opencv2/opencv.hpp>
#include"libCamera/Camera.hpp"
#include"libVision/ArmorDetect/ArmorDetect.hpp"
#include"libBase/FirstProcess.hpp"
#include"libBase/SecondProcess.hpp"
#include"libHardware/Transport/Serial.hpp"
#include"libBase/FirstProcess.hpp"
#include <iostream>
#include <pthread.h>
#include <mutex>
#include <string>

using namespace wlx;
bool button1 = false;
bool button2 = false;
int count = 10;

void* func1(void* args){
        Cam p0;
        wlx::ArmorDetect d0;
        AngleSolver A0;
        Mat pre_Frame;
        Mat curr_Frame;
        double x_lastpos,y_lastpos,z_lastpos;
        double x_speed;
        bool detect_flag = false;
        char SendMass[10];
        //while(!button);
        pre_Frame = p0.getImage();
        d0.findArmorPoint(p0.InitImg,1);
        //A0.solveAngles(d0.ARMOR_POINTS_3D,d0.armorImagePoints,d0.ArmorCenter);

        kalmanFilter KF;
        KF.Init(A0.PNP.x,500,1);
        KF.Init(A0.PNP.y,500,2);
        KF.Init(A0.PNP.z,500,3);//unsure whether dd/AS is redefined
        x_lastpos = A0.PNP.x;
        y_lastpos = A0.PNP.y;
        z_lastpos = A0.PNP.z;
        while (1)
        {
            
            if(button1){
            Cam pp;
            Mat pre_Frame;
            wlx::ArmorDetect dd;
            AngleSolver AS;
            bool detect_flag = false;
            curr_Frame = pp.getImage();
            dd.findArmorPoint(pp.InitImg,1);
            AS.solveAngles(dd.ARMOR_POINTS_3D,dd.armorImagePoints,dd.ArmorCenter);
            Point3d newPNP(KF.getPredict(AS.PNP.x,1),KF.getPredict(AS.PNP.y,2),KF.getPredict(AS.PNP.z,3));
            //cout<<"KF:"<<KF.getPredict(AS.PNP.x,1)<<KF.getPredict(AS.PNP.y,2)<<KF.getPredict(AS.PNP.z,3)<<endl;
            //std::cout<<"AS.newPNP:"<<AS.newPNP[0]<<endl;
            AS.getAngle(AS.PNP);
            Serial(AS.y_yaw,AS.x_pitch,0,1);
            //circle(curr_Frame, center, 10, Scalar(255, 255, 0), 4);
            //serial code is expected to be updated


            //putText(oriimg,std::to_string(x_speed),Point(center.x+50,center.y+50),cv::FONT_HERSHEY_PLAIN,2,(255,0,0),2);
            //dd.ArmorCenter.clear();
            pre_Frame = curr_Frame;

            }

            if(!button2){
                Cam p2;
                Mat pre_Frame;
                wlx::ArmorDetect d2;
                AngleSolver AS2;
                bool detect_flag = false;
                curr_Frame = p2.getImage();
                d2.findArmorPoint(p2.InitImg,1);
                AS2.solveAngles(d2.ARMOR_POINTS_3D,d2.armorImagePoints,d2.ArmorCenter);
                AS2.getAngle(AS2.PNP);
                Serial(AS2.y_yaw,AS2.x_pitch,0,1);
            }
            cv::waitKey(100);
        }
    return 0;
}
void* func2(void* args){

    return 0;

}

void* func3(void* args){

    return 0;
}



int main() {
    pthread_t tids[5];//一共5个线程
    //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
    int ret1 = pthread_create(&tids[0], NULL, func1, NULL);
    if (ret1 != 0)
    {
        std::cout << "pthread_create error: error_code=" << ret1 << std::endl;
    }
    int ret2 = pthread_create(&tids[1], NULL, func2, NULL);
    if (ret2 != 0)
    {
        std::cout << "pthread_create error: error_code=" << ret2 << std::endl;
    }
    int ret3 = pthread_create(&tids[2], NULL, func3, NULL);
    if (ret3 != 0)
    {
        std::cout << "pthread_create error: error_code=" << ret3 << std::endl;
    }
    //等各个线程退出后，进程才结束，否则进程强制结束了，线程可能还没反应过来；
    pthread_exit(NULL);
    return 0;
}