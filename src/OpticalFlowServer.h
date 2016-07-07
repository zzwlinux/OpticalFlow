#ifndef OPTICALFLOWSERVER_H
#define OPTICALFLOWSERVER_H
#include "OpticalFlow.h"
#include "FlightControl.h"
#include <network/InternetTransfer.h>

class OpticalFlowServer
{
public:
    OpticalFlowServer();

    void run();

    SPtr<FlightControl>                         uav;
    SPtr<Sonar>                                 sonar;
    Camera                                      camera;
    OpticalFlow                                 opticalFlow;
    SPtr<InternetTransfer<OpticalFlowResult> >  ResultTransfer;
};

#endif // OPTICALFLOWSERVER_H
