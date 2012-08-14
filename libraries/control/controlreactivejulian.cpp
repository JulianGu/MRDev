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
	maxDistanceObstacle=0.4;	//Tipically 0.6; if we use the simulator, set limit to 0.4
	leftObstacle=false;
	frontObstacle=false;
	rightObstacle=false;
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
//Copy LaserData to ControlReactive
void ControlReactiveJulian::setLaserData(LaserData laserData)
{
	this->laserData=laserData;
}
//Copy pose to ControlReactive
void ControlReactiveJulian::setPoseData(Pose3D pose)
{
	this->pose=pose;
}
//Calculate the obstacles
bool ControlReactiveJulian::compute(void)
{
	init();
	computeLaserData();
	if(minLeftRange<maxDistanceObstacle)
		leftObstacle=true;
	if(minFrontRange<maxDistanceObstacle)
		frontObstacle=true;
	if(minRightRange<maxDistanceObstacle)
		rightObstacle=true;

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
void ControlReactiveJulian::getObstaclesDistances(bool& leftObstacle, bool& frontObstacle, bool& rightObstacle, double& minLeftRange, double& minRightRange)
{
	leftObstacle= this->leftObstacle;
	frontObstacle= this->frontObstacle;
	rightObstacle= this->rightObstacle;
	minLeftRange= this->minLeftRange;
	minRightRange= this->minRightRange;
}
