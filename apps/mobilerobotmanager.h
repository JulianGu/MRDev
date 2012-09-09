#pragma once

#include "mrcore/mrcore.h"

#include <iostream>
#include <vector>
#include "localizer.h"
#include "controlmanager.h"
#include "controlmanagerjulian.h"

using namespace mr;
using namespace std;

class MobileRobotManager
{
public:
	bool drawRobot;
	MobileRobot* robot;
	MobileRobot* robotViz;
	Localizer localizer;
	ControlManagerJulian control;
	Path3D groundTraj;

	MobileRobotManager(){
		drawRobot=true;
		robot=0;
		robotViz=0;
		groundTraj.setColor(0,255,0);
	}
	virtual ~MobileRobotManager(){
		delete robot;
		delete robotViz;
	}

	Transformation3D getRobotPose(){return localizer.getEstimatedPose();}
	void drawGL(){
		control.drawGL();
		glPushMatrix();
		glTranslatef(0,0,0.01);
		groundTraj.drawGL();
		glPopMatrix();
		localizer.drawGL();
		if(robotViz && drawRobot)
		{
			robotViz->drawGL();
		}
	}
	void keyDown(unsigned char key)
	{
		if (key=='r')
			drawRobot=!drawRobot;
		else
			control.keyDown(key);
		
	}
	void processAll();
	bool step();		
	void controlStep();
	bool loadConfig(string filename);
	float getError();
	Pose3D realPose;
};
