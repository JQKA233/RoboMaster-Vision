# 华南理工大学超星五灵侠队XBOTCON无限机甲杯视觉代码WLXRoboVision
---

## 致谢  
首先在开头感谢吉林大学2020年开源代码以及西北工业大学2020年开源代码对本套代码的完成提供的巨大帮助。

---
## 介绍  
本代码是华南理工大学超星五灵侠队XBOTCON无限机甲杯2022视觉算法，主要模块分为**装甲板识别**、**位置预测**、**角度解算**、**相机驱动**及**串口/CAN通信**。  

---
## 目录
* [1. 功能介绍](#1功能介绍)
* [2. 效果展示](#2效果展示)
* [3. 依赖环境](#3依赖环境)
* [4. 整体框架](#4整体框架)
* [5. 实现方案](#5实现方案)
* [6. 通讯协议](#6通信协议)
* [7. 配置与调试](#7配置与调试)
* [8. 总结展望](#8展望)
---
## 1.功能介绍
|模块     |功能     |
| ------- | ------ |
|装甲板识别| 检测敌方机器人装甲板位置信息 |
|位置预测| 自瞄时检测移动靶 |
|角度解算| 根据上述位置信息解算目标相对枪管的yaw、pitch角度及距离 |
|相机驱动| 实现相机参数控制及图像采集 |
|串口/CAN通信| 与下位机通信，传输机器人姿态信息及操作手反馈视觉的控制信息 |
---
## 2.效果展示
### 装甲板识别
装甲板识别采用基于OpenCV的传统算法实现装甲板位置检测。因为读取的是彩色图，直方图均衡化需要在HSV空间做,同时开操作 (去除一些噪点)，闭操作 (连接一些连通域)。
识别得到装甲板在图像中四个顶点、中心点的坐标信息。  

 
### 位置预测  
使用了卡尔曼滤波分别对装甲板像素二位点进行位置更新，在自瞄状态下将自动代入解算。

 
### 角度解算  
角度解算方面使用了P4P算法。  
此外还引入了相机-枪口的Y轴距离补偿及重力补偿。  
 
---
## 3.依赖环境
### 硬件设备
|硬件|型号|参数|
|---|---|---|
|运算平台|Manifold2-G|Tx2|
|相机|USB摄像头|720p 30帧 4.2mm 70°|（初期用matlab测得相机内参矩阵和畸变矩阵，便于SolvePNP解算）
### 软件设备
|软件类型|型号|
|---|---|
|OS|Ubuntu20.04|
|Library|OpenCV-3.4.0|
---
## 4.整体框架
### 文件树  
```
 ├── build
 │   ├── CMakeCache.txt
 │   ├── CMakeFiles
 │   │   ├── 3.16.3
 │   │   │   ├── CMakeCXXCompiler.cmake
 │   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
 │   │   │   ├── CMakeSystem.cmake
 │   │   │   └── CompilerIdCXX
 │   │   │       ├── a.out
 │   │   │       ├── CMakeCXXCompilerId.cpp
 │   │   │       └── tmp
 │   │   ├── cmake.check_cache
 │   │   ├── CMakeDirectoryInformation.cmake
 │   │   ├── CMakeOutput.log
 │   │   ├── CMakeTmp
 │   │   ├── Makefile2
 │   │   ├── Makefile.cmake
 │   │   ├── progress.marks
 │   │   ├── TargetDirectories.txt
 │   │   └── WLX.dir
 │   │       ├── build.make
 │   │       ├── cmake_clean.cmake
 │   │       ├── CXX.includecache
 │   │       ├── DependInfo.cmake
 │   │       ├── depend.internal
 │   │       ├── depend.make
 │   │       ├── flags.make
 │   │       ├── libBase
 │   │       │   ├── Common.cpp.o
 │   │       │   ├── FirstProcess.cpp.o
 │   │       │   └── SecondProcess.cpp.o
 │   │       ├── libCamera
 │   │       │   └── Camera.cpp.o
 │   │       ├── libHardware
 │   │       │   └── Transport
 │   │       │       └── Serial.cpp.o
 │   │       ├── libSolver
 │   │       │   ├── Pose
 │   │       │   │   ├── AngleSolver.cpp.o
 │   │       │   │   └── Pose.cpp.o
 │   │       │   └── Predict
 │   │       │       ├── kPredict.cpp.o
 │   │       │       └── Predict.cpp.o
 │   │       ├── libVision
 │   │       │   ├── ArmorDetect
 │   │       │   │   └── ArmorDetect.cpp.o
 │   │       │   ├── FuwenDetect
 │   │       │   │   └── FuwenDetect.cpp.o
 │   │       │   └── MonoDistance
 │   │       │       └── Mono.cpp.o
 │   │       ├── link.txt
 │   │       ├── Main.cpp.o
 │   │       └── progress.make
 │   ├── cmake_install.cmake
 │   ├── compile_commands.json
 │   ├── Makefile
 │   └── WLX
 ├── CMakeLists.txt
 ├── Config
 ├── libBase
 │   ├── Common.cpp
 │   ├── Common.hpp
 │   ├── FirstProcess.cpp
 │   ├── FirstProcess.hpp
 │   ├── SecondProcess.cpp
 │   └── SecondProcess.hpp
 ├── libCamera
 │   ├── Camera.cpp
 │   └── Camera.hpp
 ├── libHardware
 │   ├── ServoControl
 │   └── Transport
 │       ├── Serial.cpp
 │       └── Serial.hpp
 ├── libSolver
 │   ├── Pose
 │   │   ├── AngleSolver.cpp
 │   │   ├── AngleSolver.hpp
 │   │   ├── Pose.cpp
 │   │   └── Pose.hpp
 │   └── Predict
 │       ├── KF.cpp
 │       ├── kPredict.cpp
 │       └── kPredict.hpp
 ├── libVision
 │   ├── ArmorDetect
 │   │   ├── ArmorDetect.cpp
 │   │   └── ArmorDetect.hpp
 │   ├── FuwenDetect
 │   │   ├── FuwenDetect.cpp
 │   │   └── FuwenDetect.hpp
 │   └── MonoDistance
 │       ├── Mono.cpp
 │       └── Mono.hpp
 ├── Main.cpp
 ├── README
 ├── setup.sh
 ├── Test
 │   ├── CameraAdjustion
 │   │   ├── build
 │   │   │   ├── CMakeCache.txt
 │   │   │   ├── CMakeFiles
 │   │   │   │   ├── 3.16.3
 │   │   │   │   │   ├── CMakeCXXCompiler.cmake
 │   │   │   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
 │   │   │   │   │   ├── CMakeSystem.cmake
 │   │   │   │   │   └── CompilerIdCXX
 │   │   │   │   │       ├── a.out
 │   │   │   │   │       ├── CMakeCXXCompilerId.cpp
 │   │   │   │   │       └── tmp
 │   │   │   │   ├── cmake.check_cache
 │   │   │   │   ├── CMakeDirectoryInformation.cmake
 │   │   │   │   ├── CMakeOutput.log
 │   │   │   │   ├── CMakeTmp
 │   │   │   │   ├── Makefile2
 │   │   │   │   ├── Makefile.cmake
 │   │   │   │   ├── progress.marks
 │   │   │   │   ├── TargetDirectories.txt
 │   │   │   │   └── Test.dir
 │   │   │   │       ├── build.make
 │   │   │   │       ├── CameraAdjustion.cpp.o
 │   │   │   │       ├── cmake_clean.cmake
 │   │   │   │       ├── CXX.includecache
 │   │   │   │       ├── DependInfo.cmake
 │   │   │   │       ├── depend.internal
 │   │   │   │       ├── depend.make
 │   │   │   │       ├── flags.make
 │   │   │   │       ├── link.txt
 │   │   │   │       └── progress.make
 │   │   │   ├── cmake_install.cmake
 │   │   │   ├── compile_commands.json
 │   │   │   ├── Makefile
 │   │   │   └── Test
 │   │   ├── CameraAdjustion.cpp
 │   │   └── CMakeLists.txt
 │   └── getTestData
 │       ├── 10.jpg
 │       ├── 11.jpg
 │       ├── 12.jpg
 │       ├── 13.jpg
 │       ├── 14.jpg
 │       ├── 15.jpg
 │       ├── 16.jpg
 │       ├── 17.jpg
 │       ├── 18.jpg
 │       ├── 19.jpg
 │       ├── 1.jpg
 │       ├── 20.jpg
 │       ├── 21.jpg
 │       ├── 22.jpg
 │       ├── 23.jpg
 │       ├── 24.jpg
 │       ├── 25.jpg
 │       ├── 26.jpg
 │       ├── 27.jpg
 │       ├── 28.jpg
 │       ├── 29.jpg
 │       ├── 2.jpg
 │       ├── 3.jpg
 │       ├── 4.jpg
 │       ├── 5.jpg
 │       ├── 6.jpg
 │       ├── 7.jpg
 │       ├── 8.jpg
 │       ├── 9.jpg
 │       ├── build
 │       │   ├── CMakeCache.txt
 │       │   ├── CMakeFiles
 │       │   │   ├── 3.16.3
 │       │   │   │   ├── CMakeCXXCompiler.cmake
 │       │   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
 │       │   │   │   ├── CMakeSystem.cmake
 │       │   │   │   └── CompilerIdCXX
 │       │   │   │       ├── a.out
 │       │   │   │       ├── CMakeCXXCompilerId.cpp
 │       │   │   │       └── tmp
 │       │   │   ├── cmake.check_cache
 │       │   │   ├── CMakeDirectoryInformation.cmake
 │       │   │   ├── CMakeOutput.log
 │       │   │   ├── CMakeTmp
 │       │   │   ├── Makefile2
 │       │   │   ├── Makefile.cmake
 │       │   │   ├── progress.marks
 │       │   │   ├── TargetDirectories.txt
 │       │   │   └── Test.dir
 │       │   │       ├── build.make
 │       │   │       ├── cmake_clean.cmake
 │       │   │       ├── CXX.includecache
 │       │   │       ├── DependInfo.cmake
 │       │   │       ├── depend.internal
 │       │   │       ├── depend.make
 │       │   │       ├── flags.make
 │       │   │       ├── getTestData.cpp.o
 │       │   │       ├── link.txt
 │       │   │       └── progress.make
 │       │   ├── cmake_install.cmake
 │       │   ├── compile_commands.json
 │       │   ├── Makefile
 │       │   └── Test
 │       ├── CMakeLists.txt
 │       └── getTestData.cpp
 └── z-help
     ├── AngleSolver.txt
     └── solvePNP
 

## 5.实现方案  
### 装甲板识别  
装甲板识别使用基于检测目标特征的OpenCV传统方法，实现检测识别的中心思想是找出图像中所有敌方颜色灯条，并使用找出的灯条一一拟合并筛选装甲板。  
主要步骤分为：**图像预处理**、**灯条检测**、**装甲板匹配**、**装甲板数字识别**及最终的**目标装甲板选择**。  
1. **图像预处理**  
为检测灯条，需要进行颜色提取。颜色提取基本思路有BGR、HSV、通道相减法。  
因为读取的是彩色图，直方图均衡化需要在HSV空间做,同时开操作 (去除一些噪点)，闭操作 (连接一些连通域)。

2. **灯条检测**  
灯条检测主要是先对预处理后的二值图找轮廓（findContours），    
使用得到的旋转矩形（RotatedRect）构造灯条。

3. **装甲板匹配**  
分析装甲板特征可知，装甲板由两个长度相等互相平行的侧面灯条构成。
我们对检测到的灯条进行两两匹配，解算出中心点，得到灯条在相机面四个角点的坐标（ImagePoints），同时以枪管中心为原点、装甲板长宽构造世界坐标系（ObjectPoints），方便代入PNP解算。

---

### 角度解算  
角度解算部分使用了两种模型解算枪管直指向目标装甲板所需旋转的yaw和pitch角。  

---
## 6.通讯协议  
以g开始为pich轴参数
以f开头为yaw参数，向单片机输送
单片机对g和f进行读取后四位之后为角度信息
单片机直接接收角度信息即可控制步进电机运动
> * button1为自瞄开启（使用卡尔曼滤波），button2为正常识别
---
## 7.配置与调试
### 运行平台搭建  
1. OpenCV库安装及配置
2. 使用systemback进行Linux系统移植
3.使用xml文件对相机参数进行调参

---

### 问题  
1. 卡尔曼滤波预测
预测算法仍待优化。
调参窗口仍待优化。
本套代码创建了许多未写的模块，在比赛的最后阶段有望更新。
