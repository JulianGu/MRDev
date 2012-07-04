#include "controltrajectoryjulian.h"
#include <float.h>


void ControlTrajectoryJulian::getSpeed(float& forward,float& turn)
{
	forward=speed;
	turn=rot;
}
void ControlTrajectoryJulian::setOdometryData(Odometry& odom)
{
	this->odom=odom;
}