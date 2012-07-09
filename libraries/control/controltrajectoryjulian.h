#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>


using namespace mr;
using namespace std;

class ControlTrajectoryJulian
{
public:
	ControlTrajectoryJulian();
	virtual bool getSpeed(float& forward,float& turn);
	void setPoseData(Pose3D& pose);
	void setNextGoal(int next);
	void setErrors(float degrees=5, float meters=0.2);
	void drawGL(void);
	
protected:
	bool computeSpeed();

private:
	float speed,rot;
	Pose3D pose;
	Path2D path;
	int nextGoal;
	float maxAngleError, maxDistanceError;
};
