#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H
#include <sys/types.h>
#include <base/types/SPtr.h>

//Type 0x11
struct UAVData//21 byte
{
    UAVData():time(0){}
    float getYaw(){return yaw*0.1f;}
    float getPitch(){return pitch*0.01f;}
    float getRoll(){return roll*0.01f;}

    float getWx(){return wx*0.01f;}
    float getWy(){return wy*0.01f;}
    float getWz(){return wz*0.01f;}

    float getLat(){return lat*1e-7;}
    float getLng(){return lng*1e-7;}
    float getAlt(){return alt*0.01f;}
    float getHDOP(){return HDOP*0.01f;}

    void  fromBuf(unsigned char* buf);
    void  toBuf(unsigned char* buf);

    int64_t time;
    int16_t yaw,pitch,roll;//*0.01 degree, 6 byte
    int16_t wx,wy,wz;      //            , 6 byte
    int32_t lat,lng;       //*1e-7 degree, 8 byte
    int16_t alt,HDOP;      //*0.01 meter,  4 byte
    u_char  nSat,fixQuality,startAutoLand; //3 byte
};

class FlightControl
{
public:
    FlightControl();
    ~FlightControl(){}

    virtual bool get(UAVData& dt){return false;}
    virtual bool valid(){return false;}
};

class FlightControlInnnoImpl;
class FlightControlInnno : public FlightControl
{
public:
    FlightControlInnno();
    virtual bool get(UAVData &dt);
    virtual bool valid();

    SPtr<FlightControlInnnoImpl> impl;
};


int UAVTest();
#endif // FLIGHTCONTROL_H
