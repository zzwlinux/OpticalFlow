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

    std::string nodeName=svar.GetString("ResultServer.NodeName","Master");
    if(nodeName.size())
    {
        ResultTransfer=SPtr<InternetTransfer<OpticalFlowResult> >(new InternetTransfer<OpticalFlowResult>());
        if( ResultTransfer->begin(nodeName) != 0 ) {
            ResultTransfer=SPtr<InternetTransfer<OpticalFlowResult> >();
        }
    }

    opticalFlow=OpticalFlow(camera,sonar,svar.GetInt("OpticalFlowType",1));

    OpticalFlowResult result;
    UAVData           uavData;
    pi::SO3f curR, curR_tmp;
    float x = 0.0f, y = 0.0f;
   // pi::SO3f camera2UAV(pi::Point3d(1,0,0),-M_PI);
    pi::SO3f UAV2camera(pi::Point3d(1,0,0),-M_PI);// = camera2UAV.inv();
    while(true)
    {
        cv::Mat img;
        video>>img;
	cv::cvtColor(img,img,CV_BGR2GRAY);
        assert(img.channels()==1);
        if(uav->get(uavData))
        {
            curR_tmp.FromEulerAngle(uavData.getPitch(), uavData.getYaw(), uavData.getRoll());
	    curR = curR_tmp*UAV2camera;
            result=opticalFlow.handleFrame(img,curR);
            x += result.x;
	    y += result.y;
	    cout<<x<<"     "<<y<<"\n";;
        }
	else{
	 printf("error");
	}
    }
}


