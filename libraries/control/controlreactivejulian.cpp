#include "controlreactivejulian.h"
#include <float.h>

void ControlReactiveJulian::getSpeed(float& forward,float& turn)
{
	forward=speed;
	turn=rot;
}
void ControlReactiveJulian::setLaserData(LaserData& laserData)
{
	this->laserData=laserData;
}