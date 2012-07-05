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
	virtual void getSpeed(float& forward,float& turn);
	void setOdometryData(Odometry& odom);
	
protected:
	void computeSpeed();

private:
	float speed,rot;
	Odometry odom;
	vector<Vector2D> path;
	int nextGoal;
	bool end;
};
