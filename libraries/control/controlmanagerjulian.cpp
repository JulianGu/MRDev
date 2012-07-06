#include "controlmanagerjulian.h"
#include <float.h>
ControlManagerJulian::ControlManagerJulian()
{
	manualControl=true;
	rot=speed=0;
	maxSpeed=2;
	maxRot=1;
}
void ControlManagerJulian::getSpeed(float& forward,float& turn)
{
	computeSpeed(forward,turn);
}
void ControlManagerJulian::keyDown(unsigned char key)
{
	if(key=='a')
		rot+=0.05f;
	else if(key=='d')
		rot-=0.05f;
	else if(key=='s')
		speed-=0.05f;
	else if(key=='w')
		speed+=0.05f;
	else if(key=='c')
	{
		manualControl=!manualControl;
		trajFollow.setNextGoal(0);
	}
	else 
	{
		speed=rot=0;
	}
}
void ControlManagerJulian::setPoseData(Pose3D& pose)
{
	this->pose=pose;
	trajFollow.setPoseData(pose);
}
void ControlManagerJulian::setLaserData(LaserData& laserData)
{
	this->laserData=laserData;
	reactive.setLaserData(laserData);
}
void ControlManagerJulian::computeSpeed(float& forward,float& turn)
{
	if(!manualControl)
	{
		manualControl=!trajFollow.getSpeed(speed,rot);
		//reactive.getSpeed(speed,rot);
	}

	if(speed>maxSpeed)speed=maxSpeed;
	if(speed<-maxSpeed)speed=-maxSpeed;
	if(rot>maxRot)rot=maxRot;
	if(rot<-maxRot)rot=-maxRot;

	forward=speed;
	turn=rot;
}
void ControlManagerJulian::drawGL(void)
{
	trajFollow.drawGL();
}