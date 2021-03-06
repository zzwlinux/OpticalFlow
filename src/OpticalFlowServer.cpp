﻿#include <time.h>
#include <fstream>
#include "OpticalFlowServer.h"
#include <network/InternetTransfer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

OpticalFlowServer::OpticalFlowServer()
{
}

void OpticalFlowServer::run()
{
    cv::VideoCapture video(0);
    if(!video.isOpened())
    {
        cerr<<"Can't open video!\n";
        return ;
    }

    camera = pi::hardware::Camera(svar.GetString("Camera","INNNO"));
    if(!camera.isValid())
    {
        cerr<<"Camera not valid.\n";
        return;
    }

    sonar=SPtr<Sonar>(new SonarUart());
    if(!sonar->valid())
    {
        cerr<<"Sonar not valid.\n";
        return ;
    }

    uav=SPtr<FlightControl>(new FlightControlInnno());
    if(!uav->valid())
    {
        cerr<<"FlightControl not valid!\n";
        return ;
    }

#ifdef INTERNET_DEBUG
    std::string nodeName=svar.GetString("ResultServer.NodeName","Master");
    if(nodeName.size())
    {
        ResultTransfer=SPtr<InternetTransfer<OpticalFlowResult> >(new InternetTransfer<OpticalFlowResult>());
        if( ResultTransfer->begin(nodeName) != 0 ) {
            ResultTransfer=SPtr<InternetTransfer<OpticalFlowResult> >();
        }
    }
#endif
    opticalFlow=OpticalFlow(camera,sonar,svar.GetInt("OpticalFlowType",1));

    OpticalFlowResult result;
    UAVData           uavData;
    pi::SO3f curR, curR_tmp;
    float x = 0.0f, y = 0.0f;
    pi::SO3f UAV2camera(pi::Point3d(1,0,0),-M_PI);
    float lastHeight = 0.0f;

    cv::Mat img;


    while(true)
    {
        video>>img;
        cv::cvtColor(img,img,CV_BGR2GRAY);
        assert(img.channels()==1);
        if(uav->get(uavData))
        {
            float currHeight = uavData.getAlt();

            float distance_ = currHeight - lastHeight;
            lastHeight = currHeight;
            curR_tmp.FromEulerAngle(uavData.getPitch(), uavData.getYaw(), uavData.getRoll());
            curR = curR_tmp*UAV2camera;

            result=opticalFlow.handleFrame(img,curR,distance_);

            x += result.x;
            y += result.y;

            MSG_INNNOSend msg;
            msg.setX(x*100);
            msg.setY(y*100);
            msg.setZ(result.h*100);

            msg.setVX(result.x*100);
            msg.setVY(result.y*100);

            uav->send(msg);
        }
        else{
            printf("error");
        }
    }
}


