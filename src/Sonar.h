#ifndef SONAR_H
#define SONAR_H
#include <base/types/SPtr.h>
#include <base/time/Time.h>
#include <base/system/thread/ThreadBase.h>
#include <deque>

class Kalman
{
public:
    Kalman();
    ~Kalman(){}

    float filter(float &measure_);

private:
    /* kalman filter states */
    float      		 x_pred ; // m
    float      		 v_pred ;
    float      		 x_post ; // m
    float    	  	 v_post ; // m/s
    struct timeval	 last ;
};

typedef uint64_t TimeStamp;
struct SonarData
{
    SonarData(float dis=0,uint64_t time=0)
        :distance(dis),timestamp(time){}
    float     distance;
    TimeStamp timestamp;
};

class Sonar
{
public:
    Sonar(int maxQueSize=1)
        :maxSize(maxQueSize){}
    virtual ~Sonar(){}
    virtual bool valid(){return false;}

    SonarData get()
    {
        pi::ScopedMutex lock(mutex);
        if(data.size()){
            return data.back();
        }
        else{
            return SonarData();
        }
    }

    bool   insert(const SonarData& f){
        pi::ScopedMutex lock(mutex);
        data.push_back(f);
    }
    size_t    size(){return data.size();}

    pi::Mutex               mutex;
    int                     maxSize;
    std::deque<SonarData>   data;
};

class SonarUartImpl;
class SonarUart : public Sonar, public pi::Thread
{
public:
    SonarUart();
    ~SonarUart();

    virtual void run();
    virtual bool valid();

    SPtr<SonarUartImpl> impl;
};


int SonarTest();

#endif // SONAR_H
