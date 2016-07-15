#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H
#include <sys/types.h>
#include <base/types/SPtr.h>
#include <ostream>
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

//Type 0x12
struct MSG_INNNOSend//12 byte
{

    void  fromBuf(unsigned char* buf)
    {
        //high first
        unsigned char* p=buf;
        x=(int16_t)(*(p++) << 8 | *(p++));
        y=(int16_t)(*(p++) << 8 | *(p++));
        z=(int16_t)(*(p++) << 8 | *(p++));

        vx=(int16_t)(*(p++) << 8 | *(p++));
        vy=(int16_t)(*(p++) << 8 | *(p++));
        vz=(int16_t)(*(p++) << 8 | *(p++));

    }

    void toBuf(unsigned char* buf)
    {
        unsigned char* p=buf;

        *(p++)=x>>8;*(p++)=x&0x00FF;
        *(p++)=y>>8;*(p++)=y&0x00FF;
        *(p++)=z>>8;*(p++)=z&0x00FF;

        *(p++)=vx>>8;*(p++)=vx&0x00FF;
        *(p++)=vy>>8;*(p++)=vy&0x00FF;
        *(p++)=vz>>8;*(p++)=vz&0x00FF;

    }

    void setX(float _x){x=_x*100;}
    void setY(float _y){y=_y*100;}
    void setZ(float _z){z=_z*100;}
    void setVX(float _x){vx=_x*100;}
    void setVY(float _y){vy=_y*100;}
    void setVZ(float _z){vz=_z*100;}

    friend inline std::ostream& operator <<(std::ostream& os,const MSG_INNNOSend& p)
    {
        os<<p.x<<" "<<p.y<<" "<<p.z<<" "
         <<p.vx<<" "<<p.vy<<" "<<p.vz;
        return os;
    }

    int16_t x,y,z,vx,vy,vz;//*0.01
};

class FlightControl
{
public:
    FlightControl();
    ~FlightControl(){}

    virtual bool get(UAVData& dt){return false;}
    virtual bool valid(){return false;}
    virtual bool send(MSG_INNNOSend& target){return false;}
};

class FlightControlInnnoImpl;
class FlightControlInnno : public FlightControl
{
public:
    FlightControlInnno();
    virtual bool get(UAVData &dt);
    virtual bool valid();
    virtual bool send(MSG_INNNOSend& target);
    SPtr<FlightControlInnnoImpl> impl;
};


int UAVTest();
#endif // FLIGHTCONTROL_H
