#include <iostream>
#include <opencv2/opencv.hpp>
#include <base/Svar/Svar.h>
#include "Sonar.h"
#include "FlightControl.h"
#include "OpticalFlowServer.h"
#include "OpticalFlowClient.h"

using namespace std;

void data_save(void){
     cv::VideoCapture video(0);
     int i = 0;
     SPtr<Sonar> sonal(new SonarUart); //声纳类
     SonarData sdata;
     FlightControlInnno uav;
     UAVData fdata;
     cv::Mat src;
     if(!video.isOpened()){
	printf("video open failed!\n");
	return ;	
     }

    for (int var = 0; var < 10; ++var) {
        video>>src;
    }
     char filename[20];
     while(1){
	usleep(1000);
	video>>src;	
	sdata=sonal->get();
        if(uav.get(fdata))
   	     printf("%f %f %f %f\n",sdata.distance,fdata.getYaw(),fdata.getPitch(),fdata.getRoll()); //声纳类， 返回采集的值	
	sprintf(filename,"./img/%5d.jpg",i++);
	if(!src.empty())
	    cv::imwrite(filename,src);
     }     

}

int main(int argc,char** argv)
{
//    printf("main!\n");
    svar.ParseMain(argc,argv);
    string act = svar.GetString("Act", "OpticalFlowServer");

    if("SonarTest" == act)   return SonarTest();             //单独测试声纳入口，看采集数据
    if("UAVTest"==act) return UAVTest();
    if("OpticalFlowServer"==act)
    {
        OpticalFlowServer server;
        server.run();
    }
    if("OpticalFlowClient"==act)
    {
        QApplication app(argc,argv);
        OpticalFlowClient client;
        return app.exec();
    }
    if("data_save" == act ) data_save();

    return 0;
}
