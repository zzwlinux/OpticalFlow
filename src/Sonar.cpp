#include "Sonar.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <base/Svar/Svar.h>

#define MAX_COM_NUM 4
#define GPIOC_BASE_ADDRESS   (0xC001C000)


Kalman::Kalman():x_pred(0.0),v_pred(0.0),
    x_post(0.0),v_post(0.0)
{
    gettimeofday(&last,0);
}

float Kalman::filter(float &measure_)
{
    struct timeval current;
    gettimeofday(&current,0);

    double dt = 0.1;//difftimeval(&current, &last);
    //printf("dt = %f\n",dt);
    last = current;
    /* no data for long time */
    if ( dt > 0.25f) // more than 2 values lost
    {
        v_pred = 0;
    }

    v_pred = v_post;
    x_pred = x_post + dt * v_post;

    x_post = x_pred + 0.8461f * (measure_ - x_pred);
    v_post = v_pred + 6.2034f * (measure_ - x_pred);
    return x_post;
}

int set_com_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits)
{
    struct termios new_cfg, old_cfg;
    int speed;

    /*保存并测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
    if(tcgetattr(fd, &old_cfg) != 0)
    {
        perror("tcgetattr error");
        return -1;
    }

    /*设置字符大小*/
    bzero(&new_cfg, sizeof(new_cfg));
    new_cfg.c_cflag |= CLOCAL | CREAD;
    new_cfg.c_cflag &= ~CSIZE;
    /*设置波特率*/
    switch(baud_rate)
    {
    case 2400:
    {
        speed = B2400;
    }
        break;
    case 4800:
    {
        speed = B4800;
    }
        break;
    case 9600:
    {
        speed = B9600;
    }
        break;
    case 19200:
    {
        speed = B19200;
    }
        break;
    case 38400:
    {
        speed = B38400;
    }
        break;
    case 57600:
    {
        speed = B57600;
    }
        break;
    default:
    case 115200:
    {
        speed = B115200;
    }
        break;
    }
    cfsetispeed(&new_cfg, speed);
    cfsetospeed(&new_cfg, speed);
    /*设置数据位*/
    switch(data_bits)
    {
    case 7:
    {
        new_cfg.c_cflag |= CS7;
    }
        break;
    default:
    case 8:
    {
        new_cfg.c_cflag |= CS8;
    }
        break;
    }
    /*设置奇偶校验位*/
    switch(parity)
    {
    default:
    case 'n':
    case 'N':
    {
        new_cfg.c_cflag &= ~PARENB;
    }
        break;
    case 'o':
    case 'O':
    {
        new_cfg.c_cflag |= (PARODD | PARENB);
        new_cfg.c_iflag |= (INPCK | ISTRIP);
    }
        break;
    case 'e':
    case 'E':
    {
        new_cfg.c_cflag |= PARENB;
        new_cfg.c_cflag &= ~PARODD;
        new_cfg.c_iflag |= (INPCK | ISTRIP);
    }
        break;
    case 's':/*as no parity*/
    case 'S':
    {
        new_cfg.c_cflag &= ~PARENB;
    }
        break;
    }
    /*设置停止位*/
    switch(stop_bits)
    {
    default:
    case 1:
    {
        new_cfg.c_cflag &= ~CSTOPB;
    }
        break;
    case 2:
    {
        new_cfg.c_cflag |= CSTOPB;
    }
        break;
    }
    /*c_cflag标志可以定义CLOCAL和CREAD，这将确保该程序不被其他端口控制和信号干扰
，同时串口驱动将读取进入的数据。CLOCAL和CREAD通常总是被是能的。*/
    /*设置等待时间和最小接收字符*/
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;
    /*处理未接收字符*/
    tcflush(fd, TCIOFLUSH);
    /*激活新配置*/
    if((tcsetattr(fd, TCSANOW, &new_cfg)) != 0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
}

/*打开串口函数*/
int open_port(int com_port)
{
    int fd;
    char *dev[] = {"/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyAMA2", "/dev/ttyAMA3"};

    if((com_port < 0) || (com_port >= MAX_COM_NUM))
    {
        return -1;
    }
    /*打开串口*/
    fd = open(dev[com_port], O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0)
    {
        perror("open serial port fail");
        return (-1);
    }
    /*恢复串口为阻塞状态*/
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        perror("fcntl failed");
        return (-1);
    }
    /*测试是否为终端设备*/
    if(isatty(STDIN_FILENO) == 0)
    {
        perror("standard input is not a terminal device");
        return (-1);
    }
    return fd;
}

class SonarUartImpl
{
public:
    SonarUartImpl():read_port_fd(-1), MAP_SIZE(40) {
        open_sonal();
    }
    ~SonarUartImpl(){
        close_sonal();
    }

    float getDistance(TimeStamp& time);

private:
    int com_init(int com_num ,int Baud);

    int open_sonal(void);

    int close_sonal(void){
        if(dev_fd)
            close(dev_fd);
        munmap(base,MAP_SIZE);
        close(read_port_fd);
    }

    int                      read_port_fd;
    int                      dev_fd;
    int                      MAP_SIZE;
    void*                    base;
};

float SonarUartImpl::getDistance(TimeStamp& time)
{
#define GPIOCOUT *(unsigned int *)base

    char buf_tmp[32] = "";
    char buf_data[32] = "";

    GPIOCOUT |= (1<<11);
    int  nread = 0;

    while (nread<=0) {
        usleep(500);
        ioctl(read_port_fd, FIONREAD, &nread);
    }
    time=pi::tm_get_us();

    float tem_buf = 0.0f;
    if(nread > 0)
    {
        char buffer[32];
        nread = read(read_port_fd, buffer, nread);
        sscanf(buffer,"%*[R]%5s",buf_tmp);
        sprintf(buf_data,"%c.%s",buf_tmp[0],&buf_tmp[1]);

        tem_buf = atof(buf_data);
        fflush(NULL);
    }
    GPIOCOUT &= ~(1<<11);
    return tem_buf;
}

int SonarUartImpl::com_init(int com_num, int Baud)
{
    int uart_fd = 0;
    if((uart_fd = open_port(com_num)) < 0)
    {
	perror("open_port");
        exit(-1);
    }
    if(set_com_config(uart_fd, Baud, 8, 'N', 1) < 0)
    {
        perror("set_com_config");
        exit(-1);
    }
    printf("\n____open uart fd is %d____\n",uart_fd);
    return uart_fd;
}

int SonarUartImpl::open_sonal(void)
{
    read_port_fd = com_init(2 ,9600);

    dev_fd = open("/dev/mem", O_RDWR | O_NDELAY);
    if (dev_fd < 0)
    {
        printf("open(/dev/mem) failed.");
        exit(-1);
    }
    base = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                MAP_SHARED, dev_fd, GPIOC_BASE_ADDRESS );

    return 0;
}

SonarUart::SonarUart()
    :impl(new SonarUartImpl())
{
    if(valid())
        start();
}

SonarUart::~SonarUart()
{
    stop();
    if(isRunning()) sleep(10);
    join();
}

bool SonarUart::valid(){return true;}

void SonarUart::run()
{
    SonarData f;
    while(!shouldStop())
    {
        f.distance=impl->getDistance(f.timestamp);
        insert(f);
    }
}

int SonarTest()
{
    SPtr<Sonar> sonal(new SonarUart); //声纳类
    SonarData data;
    u_int64_t lastTime=0;
    while(1){
        usleep(10000);
        data=sonal->get();
        if(data.distance!=0.0f)
        {
            lastTime=data.timestamp;
            printf("distance: %f\n",data.distance); //声纳类， 返回采集的值
        }
    }
    return 0;
}
