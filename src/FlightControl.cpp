#include "FlightControl.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <base/Svar/Svar.h>
#include <base/time/Time.h>
#include <hardware/UART/UART.h>
#include <string.h>

using namespace std;
typedef unsigned char uint8_t;

FlightControl::FlightControl()
{
}

void  UAVData::fromBuf(unsigned char* buf)
{
    //high first
    unsigned char* p=buf;
    yaw=(int16_t)(*(p++) << 8 | *(p++));
    pitch=(int16_t)(*(p++) << 8 | *(p++));
    roll=(int16_t)(*(p++) << 8 | *(p++));

    wx=(int16_t)(*(p++) << 8 | *(p++));
    wy=(int16_t)(*(p++) << 8 | *(p++));
    wz=(int16_t)(*(p++) << 8 | *(p++));

    lat=(int32_t)(*(p++) << 24 |*(p++) << 16 |*(p++) << 8 | *(p++));
    lng=(int32_t)(*(p++) << 24 |*(p++) << 16 |*(p++) << 8 | *(p++));
    alt=(int16_t)(*(p++) << 8 | *(p++)) ;
    HDOP=(int16_t)(*(p++) << 8 | *(p++));
    nSat=*(p++);fixQuality=*(p++);startAutoLand=*(p++);
}

void UAVData::toBuf(unsigned char* buf)
{
    unsigned char* p=buf;
    *(p++)=yaw&0xFF00;*(p++)=yaw&0x00FF;
    *(p++)=pitch&0xFF00;*(p++)=pitch&0x00FF;
    *(p++)=roll&0xFF00;*(p++)=roll&0x00FF;

    *(p++)=lat&0xFF000000;*(p++)=lat&0x00FF0000;
    *(p++)=lat&0x0000FF00;*(p++)=lat&0x000000FF;

    *(p++)=lng&0xFF000000;*(p++)=lng&0x00FF0000;
    *(p++)=lng&0x0000FF00;*(p++)=lng&0x000000FF;

    *(p++)=alt&0xFF00;*(p++)=alt&0x00FF;
    *(p++)=HDOP&0xFF00;*(p++)=HDOP&0x00FF;
    *(p++)=nSat;*(p++)=fixQuality;*(p++)=startAutoLand;
}

class FlightControlInnnoImpl : pi::Thread
{
public:
    FlightControlInnnoImpl():valid(false)
    {
        uart.port_name=svar.GetString("INNNOUAV.port","/dev/ttyAMA3");
        uart.baud_rate=svar.GetInt("INNNOUAV.baudRate",57600);
        if( 0 != uart.open() ) {
            printf("ERR: can not open port: %s (%d)\n", &uart.port_name, &uart.port_no);
            return;
        }
        valid=true;
        start();
    }
    ~FlightControlInnnoImpl()
    {
        stop();
        if(isRunning()) sleep(10);
        join();
    }

    bool get(UAVData &dt)
    {
        pi::ScopedMutex lock(mutex);
        dt=data;return true;
    }

    bool send(MSG_INNNOSend& target){
        uint8_t   sendBuf[17];
        sendBuf[0]=0xA5;sendBuf[1]=0x5A;
        sendBuf[2]=0x12;sendBuf[3]=16;

        target.toBuf(sendBuf+4);
        uint8_t sum=0;
        for(int i=2;i<16;i++)
        {
            sum+=sendBuf[i];
        }
        sendBuf[16]=sum;
        uart.write(sendBuf,17);
    }

    virtual void run()
    {
        uint8_t         buf[1024];
        int             state=0;//have obtained how many byte for the package
        uint8_t         packageType=0x00;
        int             bufIdx=0;//start point of the buf
        int             bufEnd=0;//end point of the buf
        uint8_t         typeSize=0x00;
        bool            needMore=false;

        while(!shouldStop())
        {
            //move bufIdx to 0
            if(bufIdx!=0)
            {
                memcpy(buf,buf+bufIdx,bufEnd-bufIdx);
                bufEnd-=bufIdx;
                bufIdx=0;
            }
            //obtain new buf
            {
                int received=uart.read(buf+bufEnd,1024-bufEnd);
                if(!received)
                {
                    usleep(1000);
                }
                bufEnd+=received;
                needMore=false;
            }
            //parse
            int& i=bufIdx;
            for(;i<bufEnd&&!needMore;i++)
            {
                switch(state)
                {
                case 0://find start
                    if(buf[i]==0xA5) state=1;
                    break;
                case 1://check start
                    if(buf[i]==0x5A) state=2;
                    else   state=0;
                    break;
                case 2://determin type
                    packageType=buf[i];
                    if(packageType==0x10) state=3;
                    else {
                        packageType=0x00;
                        state=0;
                    }
                    break;
                case 3://check size
                {
                    typeSize=buf[i];
                    //printf("typeSize = %d\n",typeSize);

                    int bufSize=bufEnd-bufIdx-2;
                    if(bufSize>typeSize)//wait until enough data and check CRC
                    {
                        uint8_t *data=buf+bufIdx+1;//next byte
                        uint8_t sumBytes=packageType+typeSize;
                        for(int i=0;i<typeSize;i++) sumBytes+=data[i];
                        uint8_t shouldB=sumBytes;//&0x000000FF;
                        uint8_t crc=data[typeSize];

                        if(shouldB==crc)
                        {
                            state=4;//correct
                        }
                        else
                        {
                            state=0;//wrong
                        }
                    }
                    else
                    {
                        i--;needMore=true;
                    }
                }
                    break;
                case 4://parse
                {
                    if(packageType==0x10 && typeSize==27)
                    {
                        pi::ScopedMutex lock(mutex);
                        data.time=pi::tm_get_us();
                        data.fromBuf(buf+bufIdx);
                        //bufIdx+=typeSize;//jump to CRC
                    }
                    state=0;
                }
                    break;
                default:
                    cerr<<"Not valid state, should never happens!\n";
                    break;
                }
            }
        }
    }

    pi::Mutex               mutex;
    UAVData                 data;
    pi::UART                uart;
    bool                    valid;
};

FlightControlInnno::FlightControlInnno()
    :impl(new FlightControlInnnoImpl)
{

}

bool FlightControlInnno::get(UAVData &dt)
{
    return impl->get(dt);
}

bool FlightControlInnno::valid()
{
    return impl->valid;
}

bool FlightControlInnno::send(MSG_INNNOSend &target)
{
    return impl->send(target);
}

int UAVTest()
{
    FlightControlInnno uav;
    UAVData data;
    while(1){
	printf("getdata\n");
        if(uav.get(data))
        {
            cout<<"Yaw: "<<data.getYaw()<<", "
               <<"Pitch: "<<data.getPitch()<<", "
              <<"Roll: "<<data.getRoll()<<"\n";
        }
        usleep(10000);
    }
}
