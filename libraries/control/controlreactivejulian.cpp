#include "controlreactivejulian.h"
#include <float.h>

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
void ControlReactiveJulian::setLaserData(LaserData& laserData)
{
	this->laserData=laserData;
}
void ControlReactiveJulian::setPoseData(Pose3D& pose)
{
	this->pose=pose;
}
bool ControlReactiveJulian::compute(void)
{
	init();
	computeLaserData();
	if(minLeftRange<limit)
		leftObstacle=true;
	else
		leftObstacle=false;

	if(minFrontRange<limit)
		frontObstacle=true;
	else
		frontObstacle=false;

	if(minRightRange<limit)
		rightObstacle=true;
	else
		rightObstacle=false;

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
			printf("ERROR: Robot stucked!\n");
		return true;
	}
	else
		return false;
	
}
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
		if(angles[i]<=(-30*DEG2RAD))
		{
			rightRanges.push_back(ranges[i]);
			rightAngles.push_back(angles[i]);
		}
		else if (angles[i]>(-30*DEG2RAD) && angles[i]<(30*DEG2RAD))
		{
			frontRanges.push_back(ranges[i]);
			frontAngles.push_back(angles[i]);
		}
		else if (angles[i]>=(30*DEG2RAD))
		{
			leftRanges.push_back(ranges[i]);
			leftAngles.push_back(angles[i]);
		}
		else
			printf("ERROR: Angle of LaserData out of bounds");//ERROR
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
Vector2D ControlReactiveJulian::getRightPoint(void)
{
	Vector2D point=Vector2D(pose.position.x,pose.position.y);
	double roll, pitch, yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	point.x+=dist*sin(yaw);
	point.y-=dist*cos(yaw);
	return point;
}
Vector2D ControlReactiveJulian::getLeftPoint(void)
{
	Vector2D point=Vector2D(pose.position.x,pose.position.y);
	double roll, pitch, yaw;
	pose.orientation.getRPY(roll,pitch,yaw);
	point.x-=dist*sin(yaw);
	point.y+=dist*cos(yaw);
	return point;
}