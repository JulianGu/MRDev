#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>


using namespace mr;
using namespace std;

class ControlReactiveJulian
{
public:
	ControlReactiveJulian();
	void setLaserData(LaserData laserData);
	void setPoseData(Pose3D pose);
	bool compute(void);
	void getObstaclesDistances(bool& leftObstacle, bool& frontObstacle, bool& rightObstacle, double& minLeftRange, double& minRightRange);

protected:
	void computeLaserData(void);
	void init(void);

private:
	
	LaserData laserData;
	Pose3D pose;
	double maxLeftRange,maxFrontRange,maxRightRange;
	double minLeftRange,minFrontRange,minRightRange;
	double maxDistanceObstacle;
	bool leftObstacle, frontObstacle, rightObstacle;
	
	
};
