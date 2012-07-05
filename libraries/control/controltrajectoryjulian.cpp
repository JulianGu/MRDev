#include "controltrajectoryjulian.h"
#include <float.h>

ControlTrajectoryJulian::ControlTrajectoryJulian()
{
	nextGoal=0;
	speed=0;
	rot=0;
	end=false;
	//Building path
	/*path.push_back(Vector2D(8,8));
	path.push_back(Vector2D(8,1));
	path.push_back(Vector2D(1,1));
	path.push_back(Vector2D(1,8));
	//First floor
	path.push_back(Vector2D(8,8));
	path.push_back(Vector2D(8,1));
	path.push_back(Vector2D(1,1));
	path.push_back(Vector2D(1,8));
	//Second floor
	path.push_back(Vector2D(8,8));
	path.push_back(Vector2D(8,1));
	path.push_back(Vector2D(1,1));
	path.push_back(Vector2D(1,8));
	//Third floor
	path.push_back(Vector2D(8,8));*/
	
	//Disam path
	path.push_back(Vector2D(7.8,1.5));
	path.push_back(Vector2D(8.0,3.5));
	path.push_back(Vector2D(8.8,3.6));
	path.push_back(Vector2D(6.0,1.5));
}
void ControlTrajectoryJulian::getSpeed(float& forward,float& turn)
{
	if(!end)
		computeSpeed();
	forward=speed;
	turn=rot;
}
void ControlTrajectoryJulian::setOdometryData(Odometry& odom)
{
	this->odom=odom;
}
void ControlTrajectoryJulian::computeSpeed()
{
	//Get orientation
	double roll,pitch,yaw;
	odom.pose.orientation.getRPY(roll,pitch,yaw);
	Vector2D error=path.at(nextGoal)-Vector2D(odom.pose.position.x,odom.pose.position.y);
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
	if (abs(angDiff)>=0.2)	//much orientation error
	{
		speed=0.0;
		if(angle>yaw)
			rot=0.5;
		else
			rot=-0.5;
	}
	else
	{
		rot=0.5*angDiff;
		if(error.module()>0.2)	//much position error
		{
			speed=1.0;
		}
		else	//near final point
		{
			nextGoal++;
			if(nextGoal>=path.size())	//final point
			{
				end=true;
				speed=0.0;
				rot=0.0;
			}
		}

	}
}