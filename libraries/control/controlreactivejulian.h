#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>


using namespace mr;
using namespace std;

class ControlReactiveJulian
{
public:
	virtual void getSpeed(float& forward,float& turn);
	void setLaserData(LaserData& laserData);


private:
	float speed,rot;
	LaserData laserData;
};
