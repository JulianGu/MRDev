#include "controltrajectoryjulian.h"
#include <float.h>

ControlTrajectoryJulian::ControlTrajectoryJulian()
{
	this->setErrors();
	nextGoal=0;
	speed=0;
	rot=0;
	path.setColor(255,0,255);
		
	//Disam path
	path.push_back(Vector2D(6.0,1.5));
	path.push_back(Vector2D(7.8,1.5));
	path.push_back(Vector2D(8.0,3.5));
	path.push_back(Vector2D(8.8,3.6));
	path.push_back(Vector2D(6.0,1.5));
}
bool ControlTrajectoryJulian::getSpeed(float& forward,float& turn)
{
	bool automatic=computeSpeed();
	forward=speed;
	turn=rot;
	return automatic;
}
void ControlTrajectoryJulian::setPoseData(Pose3D& pose)
{
	this->pose=pose;
}
bool ControlTrajectoryJulian::computeSpeed()
{
	//Get orientation
	double roll,pitch,yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	Vector2D error=path[nextGoal]-Vector2D(pose.position.x,pose.position.y);
	double angle=error.argument();
	//Normalization between -PI and +PI
	if(yaw>PI)
		yaw-=2*PI;
	else if(yaw<-PI)
		yaw+=2*PI;
	if(angle>PI)
		angle-=2*PI;
	else if (angle<-PI)
		angle+=2*PI;
	double angDiff=angle-yaw;
	if(angDiff>PI)
		angDiff-=2*PI;
	else if (angDiff<-PI)
		angDiff+=2*PI;

	//Cascade control
	if(error.module()<maxDistanceError)	//near final point
	{
		speed=0.0;
		rot=0.0;
		++nextGoal;
		if(nextGoal>=path.size())	//final point
			return false;
		else
			return this->computeSpeed();
	}
	if (abs(angDiff)>=maxAngleError)	//too orientation error
	{
		speed=0.0;
		if(angle>yaw)
			rot=0.5;
		else
			rot=-0.5;
	}
	else
	{
		rot=0.9*angDiff;
		speed=1.0;	//too distance error
	}
	return true;
}
void ControlTrajectoryJulian::setNextGoal(int next)
{
	this->nextGoal=next;
}
void ControlTrajectoryJulian::drawGL()
{
	path.drawGL();
}
void ControlTrajectoryJulian::setErrors(float degrees, float meters)
{
	maxAngleError=degrees*DEG2RAD;
	maxDistanceError=meters;
}