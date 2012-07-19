#include "controlreactivejulian.h"
#include <float.h>

//Constructor
ControlReactiveJulian::ControlReactiveJulian()
{
	maxLeftRange=0;
	maxFrontRange=0;
	maxRightRange=0;
	minLeftRange=100;
	minFrontRange=100;
	minRightRange=100;
	limit=0.4;	//If we are near from an obstacle the control works properly
	dist=0.5;	//Distance to the new point
	newPoint=Vector2D(0,0);
	leftObstacle=false;
	frontObstacle=false;
	rightObstacle=false;
}
//Copy LaserData to ControlReactive
void ControlReactiveJulian::setLaserData(LaserData& laserData)
{
	this->laserData=laserData;
}
//Copy pose to ControlReactive
void ControlReactiveJulian::setPoseData(Pose3D& pose)
{
	this->pose=pose;
}
//Calculate the obstacles
bool ControlReactiveJulian::compute(void)
{
	init();
	computeLaserData();
	if(minLeftRange<limit)
		leftObstacle=true;
	if(minFrontRange<limit)
		frontObstacle=true;
	if(minRightRange<limit)
		rightObstacle=true;

	if (frontObstacle)
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
	return frontObstacle;
}
//Calculate max/min distances at left/front/right
void ControlReactiveJulian::computeLaserData(void)
{
	int i=0;
	double actualAngle=laserData.getStartAngle();
	double step=laserData.getStep();
	vector<double>	ranges=laserData.getRanges();
	vector<double>	leftRanges,rightRanges,frontRanges;
	vector<Angle>	angles=laserData.getAngles();
	vector<Angle>	leftAngles,rightAngles,frontAngles;
	//Divide data into 3 packets. Angles->[-90,-30](-30,30)[30,90]
	for(int i=0;i<angles.size();i++)
	{
		if(angles[i]<=(-45*DEG2RAD))
		{
			rightRanges.push_back(ranges[i]);
			rightAngles.push_back(angles[i]);
		}
		else if (angles[i]>(-45*DEG2RAD) && angles[i]<(45*DEG2RAD))
		{
			frontRanges.push_back(ranges[i]);
			frontAngles.push_back(angles[i]);
		}
		else if (angles[i]>=(45*DEG2RAD))
		{
			leftRanges.push_back(ranges[i]);
			leftAngles.push_back(angles[i]);
		}
		else
			LOG_ERROR("ERROR: Angle of LaserData out of bounds ");
	}

	//Get max and min values
	for(i=0;i<rightRanges.size();i++)
	{
		if(rightRanges[i]>maxRightRange)
			maxRightRange=rightRanges[i];
		if(rightRanges[i]<minRightRange)
			minRightRange=rightRanges[i];
	}
	for(i=0;i<frontRanges.size();i++)
	{
		if(frontRanges[i]>maxFrontRange)
			maxFrontRange=frontRanges[i];
		if(frontRanges[i]<minFrontRange)
			minFrontRange=frontRanges[i];
	}
	for(i=0;i<leftRanges.size();i++)
	{
		if(leftRanges[i]>maxLeftRange)
			maxLeftRange=leftRanges[i];
		if(leftRanges[i]<minLeftRange)
			minLeftRange=leftRanges[i];
	}
}
//Init data every step
void ControlReactiveJulian::init(void)
{
	maxLeftRange=0;
	maxFrontRange=0;
	maxRightRange=0;
	minLeftRange=100;
	minFrontRange=100;
	minRightRange=100;
	leftObstacle=false;
	frontObstacle=false;
	rightObstacle=false;
}
//Get a good point at the right of the actual position
Vector2D ControlReactiveJulian::getRightPoint(void)
{
	Vector2D point=Vector2D(pose.position.x,pose.position.y);
	double roll, pitch, yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	if(minRightRange<=1.0)
		dist=minRightRange-(1.25*limit);
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
Vector2D ControlReactiveJulian::getLeftPoint(void)
{
	Vector2D point=Vector2D(pose.position.x,pose.position.y);
	double roll, pitch, yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	if(minLeftRange<=1.0)
		dist=minLeftRange-(1.25*limit);
	else if(minLeftRange>=2.0)
		dist=1.0;
	else
		dist=minLeftRange/2.0;
	point.x-=dist*sin(yaw);
	point.y+=dist*cos(yaw);
	LOG_INFO("New point: "<<dist<<"m (left)");
	return point;
}