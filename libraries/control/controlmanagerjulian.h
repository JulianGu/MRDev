#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>
#include "controltrajectoryjulian.h"
#include "controlreactivejulian.h"
#include "controlreplannerjulian.h"

using namespace mr;
using namespace std;

class ControlManagerJulian
{
public:
	ControlManagerJulian();
	void getSpeed(float& forward,float& turn);
	void keyDown(unsigned char key);
	void setPoseData(Pose3D& pose);
	void setLaserData(LaserData& laserData);
	void drawGL(void);

protected:
	void computeSpeed();


private:
	float speed,rot;
	float maxSpeed,maxRot;
	bool automaticControl;
	ControlTrajectoryJulian	trajectory;
	ControlReactiveJulian	reactive;
	ControlReplannerJulian	replanner;
	Pose3D		pose;
	LaserData	laserData;

};
