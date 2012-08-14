#include "controlreplannerjulian.h"
#include <float.h>

//Constructor
ControlReplannerJulian::ControlReplannerJulian()
{
	dist=0.5;	//Distance to the new point
	newPoint=Vector2D(0,0);
	minLeftRange=100;
	minRightRange=100;
	leftObstacle=false;
	frontObstacle=false;
	rightObstacle=false;
	maxDistanceObstacle=0.4;	//Tipically 0.6; if we use the simulator, set limit to 0.4
}
void ControlReplannerJulian::setLaserData(LaserData laserData)
{
	this->laserData=laserData;
}
//Copy pose to ControlReactive
void ControlReplannerJulian::setPoseData(Pose3D pose)
{
	this->pose=pose;
}
void ControlReplannerJulian::compute(void)
{
	if(leftObstacle && !rightObstacle)	//turn right
		newPoint=getRightPoint();
	else if (rightObstacle && !leftObstacle)	//turn left
		newPoint=getLeftPoint();
	else if	(!rightObstacle && !leftObstacle)//choose the best option left/right
	{
		if(minLeftRange<minRightRange)//turn right
			newPoint=getRightPoint();
		else
			newPoint=getLeftPoint();
	}
	else
		LOG_ERROR("ERROR: Robot stucked! ");
}
//Get a good point at the right of the actual position
Vector2D ControlReplannerJulian::getRightPoint(void)
{
	Vector2D point=Vector2D(pose.position.x,pose.position.y);
	double roll, pitch, yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	if(minRightRange<=1.0)
		dist=minRightRange-(1.25*maxDistanceObstacle);
	else if(minRightRange>=2.0)
		dist=1.0;
	else
		dist=minRightRange/2.0;
	point.x+=dist*sin(yaw);
	point.y-=dist*cos(yaw);
	LOG_INFO("New point: "<<dist<<"m (right)");
	return point;
}
//Get a good point at the left of the actual position
Vector2D ControlReplannerJulian::getLeftPoint(void)
{
	Vector2D point=Vector2D(pose.position.x,pose.position.y);
	double roll, pitch, yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	if(minLeftRange<=1.0)
		dist=minLeftRange-(1.25*maxDistanceObstacle);
	else if(minLeftRange>=2.0)
		dist=1.0;
	else
		dist=minLeftRange/2.0;
	point.x-=dist*sin(yaw);
	point.y+=dist*cos(yaw);
	LOG_INFO("New point: "<<dist<<"m (left)");
	return point;
}
void ControlReplannerJulian::setObstaclesDistances(bool& leftObstacle, bool& frontObstacle, bool& rightObstacle, double& minLeftRange, double& minRightRange)
{
	this->leftObstacle= leftObstacle;
	this->frontObstacle= frontObstacle;
	this->rightObstacle= rightObstacle;
	this->minLeftRange= minLeftRange;
	this->minRightRange= minRightRange;
}