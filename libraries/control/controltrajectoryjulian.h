#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>


using namespace mr;
using namespace std;

class ControlTrajectoryJulian
{
public:
	virtual void getSpeed(float& forward,float& turn);
	void setOdometryData(Odometry& odom);
	

private:
	float speed,rot;
	Odometry odom;
};
