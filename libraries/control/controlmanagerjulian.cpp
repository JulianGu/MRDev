#include "controlmanagerjulian.h"
#include <float.h>

//Constructor
ControlManagerJulian::ControlManagerJulian()
{
	automaticControl=false;
	rot=0;
	speed=0;
	maxSpeed=2;
	maxRot=1;
}
void ControlManagerJulian::getSpeed(float& forward,float& turn)
{
	computeSpeed();
	forward=speed;
	turn=rot;
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
		automaticControl=!automaticControl;
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
	reactive.setPoseData(pose);
}
void ControlManagerJulian::setLaserData(LaserData& laserData)
{
	this->laserData=laserData;
	reactive.setLaserData(laserData);
}
void ControlManagerJulian::computeSpeed()
{
	if(automaticControl)
	{
		if(trajFollow.getBlockReactive())
		{
			automaticControl=trajFollow.getSpeed(speed,rot);
		}
		else
		{
			if(reactive.compute())
				trajFollow.addPoint(reactive.getNewPoint());
			automaticControl=trajFollow.getSpeed(speed,rot);
		}
	}

	if(speed>maxSpeed)speed=maxSpeed;
	if(speed<-maxSpeed)speed=-maxSpeed;
	if(rot>maxRot)rot=maxRot;
	if(rot<-maxRot)rot=-maxRot;
}
void ControlManagerJulian::drawGL(void)
{
	trajFollow.drawGL();
}
