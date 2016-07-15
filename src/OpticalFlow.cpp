#include "OpticalFlow.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int OpticalFlowResult::toStream(pi::RDataStream &ds)
{
    ds.clear();
    // set magic number & version number
    ds.setHeader(0x83F8, 1);

    ds.write((uchar*)this,sizeof(OpticalFlowResult));
}

int OpticalFlowResult::fromStream(pi::RDataStream &ds)
{
    uint        d_magic, d_ver;

    ds.rewind();

    ds.getHeader(d_magic, d_ver);

    if( d_magic != 0x83F8 ) {
        dbg_pe("Input data magic number error! %x\n", d_magic);
        return -1;
    }
    ds.read((uchar*)this,sizeof(OpticalFlowResult));

    return 0;
}

std::ostream& operator <<(std::ostream& os,const OpticalFlowResult& rh)
{
    os<<"["<<rh.x<<" "<<rh.y<<" "<<rh.h<<" "<<rh.corrNum<<"]";
    return os;
}

class OpticalFlowImpl
{
public:
    OpticalFlowImpl(pi::hardware::Camera camera_,const SPtr<Sonar> sonar_)
        :camera(camera_),sonar(sonar_)
    {

    }

    virtual OpticalFlowResult handleFrame(const cv::Mat& curImg,const pi::SO3f& rotation,const float& distance_)
    {
        return OpticalFlowResult();
    }

    pi::hardware::Camera        camera;
    cv::Mat                     preImg;
    pi::SO3f                    preR;
    SPtr<Sonar>                 sonar;
};

class OpticalFlowWithPose:public OpticalFlowImpl
{

public:
    OpticalFlowWithPose(pi::hardware::Camera camera_,const SPtr<Sonar> sonar_)
        :OpticalFlowImpl(camera_, sonar_),preH(-1)
    {
    }

    void  flushCurrent(const cv::Mat& curImg,const pi::SO3f& rotation,float height=0)
    {
        preImg=curImg.clone();preR=rotation;
        if(!height)
        {
            float r[9];
            rotation.getMatrix(r);
	    preH=sonar->get().distance*r[8];
        }
	else {
	   preH = height;		
	}
    }

    virtual OpticalFlowResult handleFrame(const cv::Mat& curImg,const pi::SO3f& rotation,const float& distance_)
    {
        OpticalFlowResult result;
        ASSERT2(curImg.channels()==1,"OpticalFlow need gray image input!\n");
        if(preImg.empty())
        {
            flushCurrent(curImg,rotation);
            return result;
        }

        std::vector<cv::Point2f> preCorners,curCorners,preCornersGood,curCornersGood;
        vector <uchar> status;
        vector <float> err;

        cv::goodFeaturesToTrack(preImg,preCorners,40, 0.01, 30, cv::noArray(), 3, true, 0.04);

        if(preCorners.size()<3)
        {
            flushCurrent(curImg,rotation);
            return result;
        }
        calcOpticalFlowPyrLK(preImg, curImg, preCorners, curCorners, status, err);

        for(size_t i=0; i < status.size(); i++)
        {
            if(status[i])
            {
                preCornersGood.push_back(preCorners[i]);
           // cout<<"dx= "<<preCorners[i].x-curCorners[i].x<<", dy="<<preCorners[i].y-curCorners[i].y<<"\n";
                curCornersGood.push_back(curCorners[i]);
            }
        }
        status.clear();
        if(preCornersGood.size()<3)
        {
            flushCurrent(curImg,rotation);
            return result;
        }

        float r[9];
        rotation.getMatrix(r);
        float raw_dis = sonar->get().distance;
        result.h=raw_dis*r[8];

        float heightIsGood = result.h - preH - distance_;
        if(heightIsGood>0.4 || heightIsGood<-0.4)
            result.h = preH + distance_ ;

        vector<pi::Point2f> preGroundPts,curGroundPts;
        projectPoints2Ground(preCornersGood,preR,preGroundPts,preH);
        projectPoints2Ground(curCornersGood,rotation,curGroundPts,result.h);

        for(int i=0;i<preGroundPts.size();i++){
            preGroundPts[i]=curGroundPts[i]-preGroundPts[i];
        }
        //RANSAC and find dx dy
        //        findXYRansac(preGroundPts,result.x,result.y);
        result.corrNum=findXY3Sigma(preGroundPts,result.x,result.y);

        flushCurrent(curImg,rotation,result.h);


        return result;
    }

private:
    bool projectPoints2Ground(const std::vector<cv::Point2f>& pts,const pi::SO3f& rotation,vector<pi::Point2f>& groundPts,float height)
    {
        groundPts.resize(pts.size());
        for(int i=pts.size()-1;i>=0;i--)
        {
            const cv::Point2f& pt=pts[i];
            pi::Point3f axis=rotation*camera.UnProject(pi::Point2d(pt.x,pt.y));
            axis=axis*(height/axis.z);
            groundPts[i]=pi::Point2f(axis.x,axis.y);
        }

    }

    bool projectPoints2Ground(const std::vector<cv::KeyPoint>& pts,const pi::SO3f& rotation,vector<pi::Point2f>& groundPts,float height)
    {
        groundPts.resize(pts.size());
        for(int i=pts.size()-1;i>=0;i--)
        {
            const cv::Point2f& pt=pts[i].pt;
            pi::Point3f axis=rotation*camera.UnProject(pi::Point2d(pt.x,pt.y));
            axis=axis*(height/axis.z);
            groundPts[i]=pi::Point2f(axis.x,axis.y);
        }

    }


    int findXY3Sigma(vector<pi::Point2f>& pts,float& x,float& y)
    {
        // 1. find mean
        pi::Point2f sum(0,0),average;
        int   inlierCount=pts.size();
        for(int i=0;i<inlierCount;i++) sum=sum+pts[i];
        average=sum/inlierCount;

        // 2. find sigma2
        float sigma2=0;pi::Point2f tmp;
        vector<float> sigma2array;
        sigma2array.resize(inlierCount);
        for(int i=0;i<inlierCount;i++)
        {
            tmp=(pts[i]-average);
            sigma2array[i]=tmp.x*tmp.x+tmp.y*tmp.y;
            sigma2+=sigma2array[i];
        }
        sigma2=sigma2/(inlierCount-1);

        // 3. remove outliers (sigma_i^2 > 3*sigma2)
        float sigma2Threshold=sigma2*9.;
        for(int i=0;i<pts.size();i++)
            if(sigma2Threshold<=sigma2array[i])
            {
                sigma2array[i]=-1;
                inlierCount--;
                sum=sum-pts[i];
            }

        // 4. inliers mean
        if(inlierCount!=pts.size())
        {
            average=sum/inlierCount;
        }

        x=average.x;y=average.y;
        return inlierCount;
    }

    int findXYRansac(vector<pi::Point2f>& pts,float& x,float& y)
    {
    }

    float preH;
};



class OpticalFlowSimp:public OpticalFlowImpl
{
public:
    OpticalFlowSimp(pi::hardware::Camera camera_, const SPtr<Sonar> sonar_)
        :OpticalFlowImpl(camera_, sonar_)
    {
        pi::Point2d cxy=camera.Project(pi::Point3d(0,0,1));
        pi::Point2d fxy=camera.Project(pi::Point3d(1,1,1))-cxy;
        fxInv=1./fxy.x;fyInv=1./fxy.y;
        printf("fxInv = %f,fyInv = %f\n",fxInv,fyInv);
    }

    virtual OpticalFlowResult handleFrame(const cv::Mat& curImg,const pi::SO3f& rotation)
    {
        OpticalFlowResult result;
        ASSERT2(curImg.channels()==1,"OpticalFlow need gray image input!\n");

        if(preImg.empty())
        {
            preImg=curImg;
            preR=rotation;
            return result;
        }

        std::vector<cv::Point2f> preCorners,curCorners,preCornersGood,curCornersGood;
        vector <uchar> status;
        vector <float> err;

        cv::goodFeaturesToTrack(preImg,preCorners,40, 0.01, 30, cv::noArray(), 3, true, 0.04);
        calcOpticalFlowPyrLK(preImg, curImg, preCorners, curCorners, status, err);
        for(size_t i=0; i < status.size(); i++)
        {
            if(status[i])
            {
                preCornersGood.push_back(preCorners[i]);
                curCornersGood.push_back(curCorners[i]);
            }
        }
        status.clear();
        if(preCornersGood.size()<5)
            return result;

        cv::Mat H = cv::Mat::zeros(2,3,CV_64F);
        H = cv::estimateRigidTransform(preCornersGood,curCornersGood,false);
        
	double dx = H.at<double>(0,2);
        double dy = H.at<double>(1,2);
        
	result.h=sonar->get().distance;
        result.x=dx*fxInv*result.h*100;
        result.y=dy*fyInv*result.h*100;
        result.corrNum = 1;
        return result;
    }

    float fxInv,fyInv;
};

OpticalFlow::OpticalFlow(const pi::hardware::Camera& camera,const SPtr<Sonar> sonar_,int type)
    :impl(new OpticalFlowImpl(camera,sonar_))
{
    if(type==OpticalFlowTypeSimp) impl=SPtr<OpticalFlowImpl>(new OpticalFlowSimp(camera,sonar_));
    if(type==OpticalFlowTypeWithPose) impl=SPtr<OpticalFlowImpl>(new OpticalFlowWithPose(camera,sonar_));
}

OpticalFlowResult OpticalFlow::handleFrame(const Mat &curImg, const pi::SO3f &rotation, const float &distance_)
{
    return impl->handleFrame(curImg,rotation,distance_);
}

