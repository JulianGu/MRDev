#include "controltrajectoryjulian.h"
#include <float.h>

//Constructor
ControlTrajectoryJulian::ControlTrajectoryJulian()
{
	this->setErrors();
	nextGoal=0;
	speed=0;
	rot=0;
	path.setColor(153,0,204);

	//EmptyWorld path 1
	double x=8.0, y=8.0;
	/*path.push_back(Vector3D(x,y));
	path.push_back(Vector3D(x+4.0,y));
	path.push_back(Vector3D(x+4.0,y+4.0));
	path.push_back(Vector3D(x,y+4.0));
	path.push_back(Vector3D(x,y));*/

	//EmptyWorld path 2
	path.push_back(Vector3D(x,y));
	path.push_back(Vector3D(x+4.0,y));
	path.push_back(Vector3D(x+4.0,y+4.0));
	path.push_back(Vector3D(x,y+4.0));
	path.push_back(Vector3D(x,y+1.0));
	path.push_back(Vector3D(x+3.0,y+1.0));
	path.push_back(Vector3D(x+3.0,y+3.0));
	path.push_back(Vector3D(x+1.0,y+3.0));
	path.push_back(Vector3D(x+1.0,y+2.0));
	path.push_back(Vector3D(x+2.0,y+2.0));
	//Disam path
	/*double x=6.3, y=1.5;
	path.push_back(Vector3D(x,y));
	path.push_back(Vector3D(x+1.2,y));
	path.push_back(Vector3D(x+1.7,y+2));
	path.push_back(Vector3D(x+2.3,y+2));
	path.push_back(Vector3D(x+0.2,y));*/
}
bool ControlTrajectoryJulian::getSpeed(float& forward,float& turn)
{
	bool automatic=computeSpeed();
	forward=speed;
	turn=rot;
	return automatic;
}
//Copy pose to ControlTrajectory
void ControlTrajectoryJulian::setPoseData(Pose3D& pose)
{
	this->pose=pose;
}
//Calculates the speed/rot
bool ControlTrajectoryJulian::computeSpeed()
{
	//Get orientation
	double roll,pitch,yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	Vector2D error=Vector2D(path[nextGoal].x,path[nextGoal].y)-Vector2D(pose.position.x,pose.position.y);
	double wantedAng=error.argument();
	double diffAng=Angle::difference(yaw,wantedAng);
	changeRange(diffAng);
	if(error.module()<maxDistanceError)	//near final point
	{
		blockReplanner=false;
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
		blockReplanner=true;
		speed=0.0;
		if(diffAng>=0)
			rot=-0.3;
		else if(diffAng<0)
			rot=0.3;
	}
	else
	{
		blockReplanner=false;
		rot=-1.1*diffAng;
		speed=0.5;	//too distance error
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
	pose.drawGL();
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
//Add new point to the path (the next point)
void ControlTrajectoryJulian::addPoint(Vector2D newPoint)
{
	vector<Vector3D> copy(path.size());
	for(int i=0;i<copy.size();i++)
		copy[i]=path[i];
	copy.insert(copy.begin()+nextGoal,Vector3D(newPoint.x,newPoint.y));
	path=Path3D(copy);
	path.setColor(153,0,204);
}