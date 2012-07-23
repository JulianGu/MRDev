#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>


using namespace mr;
using namespace std;

class ControlReactiveJulian
{
public:
	void init(void);
	ControlReactiveJulian();
	void setLaserData(LaserData laserData);
	void setPoseData(Pose3D pose);
	Vector2D getNewPoint(void){return newPoint;}
	bool compute(void);

protected:
	void computeLaserData(void);
	Vector2D getRightPoint(void);
	Vector2D getLeftPoint(void);

private:
	float dist;
	LaserData laserData;
	Pose3D pose;
	double maxLeftRange,maxFrontRange,maxRightRange;
	double minLeftRange,minFrontRange,minRightRange;
	double limit;
	bool leftObstacle, frontObstacle, rightObstacle;
	Vector2D newPoint;
	
};
