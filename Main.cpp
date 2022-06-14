#include<opencv2/opencv.hpp>
#include"libCamera/Camera.hpp"
#include"libVision/ArmorDetect/ArmorDetect.hpp"
#include"libBase/FirstProcess.hpp"
#include"libBase/SecondProcess.hpp"
#include"libHardware/Transport/Serial.hpp"
#include <iostream>
#include <pthread.h>
#include <mutex>
#include <string>

using namespace wlx;
bool button1 = false;
bool button2 = true;
int count = 10;
char receivedMessage[8]={0,0,0,0,0,0,0,0};
int len2;  
int fd2 = -1;  
int func1()
{
    if(button1){
        Cam pp;
        Mat pre_Frame;
        wlx::ArmorDetect dd;
        AngleSolver AS;
        kalmanFilter KF;
        bool detect_flag = false;
        //pp.Cam_Init();
        pp.getImage();
        dd.findArmorPoint(pp.InitImg,1);
        if(dd.armorImagePoints.empty())
        {
            cout <<"NO POINTS FOUND"<<endl;
        }
        else 
        {
            Point2f Point1 = KF.KF(dd.armorImagePoints[0]);//predict
            Point2f Point2 = KF.KF(dd.armorImagePoints[1]);
            Point2f Point3 = KF.KF(dd.armorImagePoints[2]);
            Point2f Point4 = KF.KF(dd.armorImagePoints[3]);
            KF.setKF(Point1,Point2,Point3,Point4);
            cout<<"Kpoints"<<KF.Kpoints<<endl;
            AS.solveAngles(dd.ARMOR_POINTS_3D,KF.Kpoints,dd.ArmorCenter);

            AS.getAngle(AS.PNP);
            Serial(AS.y_yaw,AS.x_pitch,0,1);
        }
    
    }
    if(button2)
    {
        Cam p2;
        Mat pre_Frame;
        wlx::ArmorDetect d2;
        AngleSolver AS2;
        bool detect_flag = false;
        //p2.Cam_Init();
        p2.getImage();
        d2.findArmorPoint(p2.InitImg,1);
        AS2.solveAngles(d2.ARMOR_POINTS_3D,d2.armorImagePoints,d2.ArmorCenter);
        AS2.getAngle(AS2.PNP);
        Serial(AS2.y_yaw+15,AS2.x_pitch,0,1);
    }
    return 0;
}
        
    
int REIn(){
         //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug    
    int err;               //返回调用函数的状态    

    int flag=0;
    //char rcv_buf[256];             
    //char send_buf[256];
    //判断是否打开串口
    const char *dev[]  = {"/dev/ttyUSB0", "/dev/ttyUSB0"};
    fd2 = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
     if(-1 == fd2)
        {
            perror("Can't Open Serial Port");
            return 0;
        }else

     {
         flag=1;//Seral open permit to trans
     }
     //fd=open("dev/ttyS1", O_RDWR);
    //printf("fd= \n",fd);
     do
    {
        err = UART0_Init(fd2,115200,0,8,1,'N');
        //printf("Set Port Exactly!\n");
        usleep(100);
    }while(FALSE == err || FALSE == fd2);
    
}
int func2(){
    len2 = UART0_Recv(fd2,receivedMessage,8);
    if(len2>0){
        cout<<receivedMessage<<endl;
        if(receivedMessage[0]=='p'){
            cout<<receivedMessage<<endl;
            if(receivedMessage[1]=='1'&&receivedMessage[2]=='0'){
                button2 = true;
                button1 = false;
            }
            if(receivedMessage[1]=='1'&&receivedMessage[2]=='1'){
                button1 = true;
                button2 = false;
            }
            if(receivedMessage[1]=='0'){
                button1 = false;
                button2 = false;
            }
        }
    }
    return 0;
}


int main() {
    REIn();
    while (1)
    {
        func2();
        cout<<button1<<" "<<button2<<endl;
        while(button1||button2){
            func1();
        }
    }
    
    
    return 0;
}