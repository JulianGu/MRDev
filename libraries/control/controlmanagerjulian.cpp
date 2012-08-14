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
		trajectory.setNextGoal(0);
	}
	else 
	{
		speed=rot=0;
	}
}
void ControlManagerJulian::setPoseData(Pose3D& pose)
{
	this->pose=pose;
	trajectory.setPoseData(pose);
	reactive.setPoseData(pose);
	replanner.setPoseData(pose);
}
void ControlManagerJulian::setLaserData(LaserData& laserData)
{
	this->laserData=laserData;
	reactive.setLaserData(laserData);
	replanner.setLaserData(laserData);
}
void ControlManagerJulian::computeSpeed()
{
	if(automaticControl)
	{
		if(trajectory.getBlockReactive())
		{
			automaticControl=trajectory.getSpeed(speed,rot);
		}
		else
		{
			if(reactive.compute())
			{
				double minLeftRange,minRightRange;
				bool leftObstacle, frontObstacle, rightObstacle;
				reactive.getObstaclesDistances(leftObstacle, frontObstacle, rightObstacle, minLeftRange, minRightRange);
				replanner.setObstaclesDistances(leftObstacle, frontObstacle, rightObstacle, minLeftRange, minRightRange);
				replanner.compute();
				trajectory.addPoint(replanner.getNewPoint());
			}
			automaticControl=trajectory.getSpeed(speed,rot);
		}
	}

	if(speed>maxSpeed)speed=maxSpeed;
	if(speed<-maxSpeed)speed=-maxSpeed;
	if(rot>maxRot)rot=maxRot;
	if(rot<-maxRot)rot=-maxRot;
}
void ControlManagerJulian::drawGL(void)
{
	trajectory.drawGL();
}
