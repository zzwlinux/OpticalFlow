#include "OpticalFlowClient.h"
#include <GL/freeglut.h>
#include <gui/gl/glHelper.h>

OpticalFlowClient::OpticalFlowClient()
    :uavSize(svar.GetDouble("UAVSize",0.2))
{
    std::string nodeName=svar.GetString("ResultClient.NodeName","Client");
    if(nodeName.size())
    {
        ResultTransfer=SPtr<InternetTransfer<OpticalFlowResult> >(new InternetTransfer<OpticalFlowResult>());
        if( ResultTransfer->begin(nodeName) != 0 ) {
            ResultTransfer=SPtr<InternetTransfer<OpticalFlowResult> >();
            start();
        }
    }
}

OpticalFlowClient::~OpticalFlowClient()
{
    stop();
    if(isRunning()) sleep(10);
    join();
}

void OpticalFlowClient::run()
{
    win3d.Show();

    if(!shouldStop())
    {
        if(ResultTransfer.get()&&ResultTransfer->size())
        {
            pi::WriteMutex lock(mutex);
            curState=ResultTransfer->pop();
            path.push_back(curState->pose.get_translation());
        }
    }
}

void OpticalFlowClient::Draw_Something()
{
    std::vector<pi::Point3f>                    path;
    SPtr<OpticalFlowResult>                     curState;
    {
        pi::ReadMutex lock(mutex);
        path=this->path;
        curState=this->curState;
    }

    // draw history
    glColor3ub(255,255,0);
    glBegin(GL_LINE);
    for(size_t i=0;i<path.size();i++)
        glVertex(path[i]);
    glEnd();

    // draw current state here
    if(!curState.get()) return;
    if(!svar.GetInt("GlutInited"))
    {
        glutInit(&svar.i["argc"],SvarWithType<char**>::instance()["argv"]);
        svar.GetInt("GlutInited")=1;
    }
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);

    glMultMatrix(curState->pose);

    // draw the uav here
    const float length=0.3;
    glDisable(GL_LIGHTING);
    glLineWidth(2.5);
    glBegin(GL_LINES);
    glColor3ub(255,0,0);glVertex3f(0,0,0);glVertex3f(length,0,0);
    glColor3ub(0,255,0);glVertex3f(0,0,0);glVertex3f(0,length,0);
    glColor3ub(0,0,255);glVertex3f(0,0,0);glVertex3f(0,0,length);
    glEnd();

    // consider to draw the wind, this may be cool and funny, haha~
    glLineWidth(5);
    glBegin(GL_LINES);
    glColor3ub(0,255,0);glVertex3f(0.5*uavSize,0.5*uavSize,0);glVertex3f(-0.5*uavSize,-0.5*uavSize,0);
    glColor3ub(0,255,0);glVertex3f(-0.5*uavSize,0.5*uavSize,0);glVertex3f(0.5*uavSize,-0.5*uavSize,0);
    glEnd();
    glColor3ub(255,0,0); glTranslatef(0.5*uavSize,0.5*uavSize,0); glutSolidCone(0.22*uavSize,0.05*uavSize,30,2);
    glColor3ub(0,0,255); glTranslatef(-uavSize,0,0); glutSolidCone(0.22*uavSize,0.05*uavSize,30,2);
    glColor3ub(0,0,255); glTranslatef(0,-uavSize,0); glutSolidCone(0.22*uavSize,0.05*uavSize,30,2);
    glColor3ub(255,0,0); glTranslatef(uavSize,0,0);  glutSolidCone(0.22*uavSize,0.05*uavSize,30,2);

    glPopMatrix();
}


