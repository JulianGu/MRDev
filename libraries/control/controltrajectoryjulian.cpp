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
	double x=6.3, y=1.5;
	path.push_back(Vector2D(x,y));
	path.push_back(Vector2D(x+1.2,y));
	path.push_back(Vector2D(x+1.7,y+2));
	path.push_back(Vector2D(x+2.3,y+2));
	path.push_back(Vector2D(x+0.2,y));
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
	double wantedAng=error.argument();
	double diffAng=Angle::difference(yaw,wantedAng);
	changeRange(diffAng);
	if(error.module()<maxDistanceError)	//near final point
	{
		blockReactive=false;
		speed=0.0;
		rot=0.0;
		++nextGoal;
		if(nextGoal>=path.size())	//final point
			return false;
		else
			return this->computeSpeed();
	}
	if (abs(diffAng)>=maxAngleError)	//too orientation error
	{
		blockReactive=true;
		speed=0.0;
		if(diffAng>=0)
			rot=-0.3;
		else if(diffAng<0)
			rot=0.3;
	}
	else
	{
		blockReactive=false;
		rot=-0.9*diffAng;
		speed=0.3;	//too distance error
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
//Changes from [0,2*PI] to [-PI,PI]
void ControlTrajectoryJulian::changeRange(double &angle)
{
	if(angle>PI)
		angle-=2*PI;
}
void ControlTrajectoryJulian::addPoint(Vector2D newPoint)
{
	path.points.insert(path.points.begin()+nextGoal,newPoint);
}