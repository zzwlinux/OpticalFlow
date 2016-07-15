#ifndef OPTICALFLOWCLIENT_H
#define OPTICALFLOWCLIENT_H
#include "OpticalFlow.h"
#include <network/InternetTransfer.h>
#include <gui/gl/Win3D.h>

class OpticalFlowClient : public pi::Thread,public pi::gl::Draw_Opengl
{
public:
    OpticalFlowClient();
    ~OpticalFlowClient();

    virtual void run();
    virtual void Draw_Something();

    SPtr<InternetTransfer<OpticalFlowResult> >  ResultTransfer;

    pi::MutexRW                                 mutex;
    std::vector<pi::Point3f>                    path;
    SPtr<OpticalFlowResult>                     curState;
    pi::gl::Win3D                               win3d;
    float                                       uavSize;
};

#endif // OPTICALFLOWCLIENT_H
