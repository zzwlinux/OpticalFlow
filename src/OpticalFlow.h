#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H
#include <opencv2/core/core.hpp>
#include <base/types/SPtr.h>
#include <base/types/SE3.h>
#include <base/Svar/DataStream.h>
#include <hardware/Camera/Cameras.h>
#include "Sonar.h"

typedef pi::hardware::Camera Camera;

struct OpticalFlowResult
{
    OpticalFlowResult():corrNum(-1){}

    // DataStream interface
    int toStream(pi::RDataStream &ds);
    int fromStream(pi::RDataStream &ds);

    OpticalFlowResult operator +=(const OpticalFlowResult& right){
        this->x += right.x;
        this->y += right.y;
        return *this;
    }

    float       x,y,h;
    pi::SE3f    pose;
    int         corrNum;
};

std::ostream& operator <<(std::ostream& os,const OpticalFlowResult& rh);

enum OpticalFlowType{OpticalFlowTypeNone=0,OpticalFlowTypeSimp=1,OpticalFlowTypeWithPose=2};

class OpticalFlowImpl;
class OpticalFlow
{
public:
    OpticalFlow(const pi::hardware::Camera& camera=pi::hardware::Camera(),
                const SPtr<Sonar> sonar=SPtr<Sonar>(),int type=0);

    OpticalFlowResult handleFrame(const cv::Mat& curImg,const pi::SO3f& rotation,const float &distance_);

private:
    SPtr<OpticalFlowImpl> impl;
};

#endif // OPTICALFLOW_H
