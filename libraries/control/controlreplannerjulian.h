#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>

using namespace mr;
using namespace std;

class ControlReplannerJulian
{
public:
	ControlReplannerJulian();
	void setLaserData(LaserData laserData);
	void setPoseData(Pose3D pose);
	Vector2D getNewPoint(void){return newPoint;}
	void compute(void);
	void setObstaclesDistances(bool& leftObstacle, bool& frontObstacle, bool& rightObstacle, double& minLeftRange, double& minRightRange);

protected:
	Vector2D getRightPoint(void);
	Vector2D getLeftPoint(void);


private:
	float dist;
	Pose3D pose;
	LaserData laserData;
	Vector2D newPoint;
	double minLeftRange, minRightRange;
	bool leftObstacle, frontObstacle, rightObstacle;
	double maxDistanceObstacle;
};
