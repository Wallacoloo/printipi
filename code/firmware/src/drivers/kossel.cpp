#include "kossel.h"

//Maths derived from Mathematica:
/* Given vx, vy, vz, x0, y0, z0, solve for the next time at which to step axis A (two possible results. One, or both, will be negative:
  t = -(-(Sqrt(3)*d*vy) - 4*A*vz + 4*vx*x0 + 4*vy*y0 + 4*vz*z0 + 
      Sqrt(Power(Sqrt(3)*d*vy + 4*A*vz - 4*(vx*x0 + vy*y0 + vz*z0),2) - 
        (Power(vx,2) + Power(vy,2) + Power(vz,2))*(16*Power(A,2) + 3*Power(d,2) - 8*Sqrt(3)*d*y0 - 32*A*z0 + 
           16*(-Power(L,2) + Power(x0,2) + Power(y0,2) + Power(z0,2)))))/(4.*(Power(vx,2) + Power(vy,2) + Power(vz,2)))
           
  t = (Sqrt(3)*d*vy + 4*A*vz - 4*vx*x0 - 4*vy*y0 - 4*vz*z0 + 
     Sqrt(Power(Sqrt(3)*d*vy + 4*A*vz - 4*(vx*x0 + vy*y0 + vz*z0),2) - 
       (Power(vx,2) + Power(vy,2) + Power(vz,2))*(16*Power(A,2) + 3*Power(d,2) - 8*Sqrt(3)*d*y0 - 32*A*z0 + 
          16*(-Power(L,2) + Power(x0,2) + Power(y0,2) + Power(z0,2)))))/(4.*(Power(vx,2) + Power(vy,2) + Power(vz,2)))*/

namespace drv {

void Kossel::getTemperature(int &extruder, int &bed) const {
	extruder=100;
	//bed=100;
}

std::size_t Kossel::numAxis() const {
	return 4; //A, B, C + Extruder
}

float Kossel::defaultMoveRate() const {
	return 10;
}
float Kossel::defaultFeedRate() const {
	return 10;
}

float Kossel::relativeTimeOfNextStep(int axisIdx, gparse::StepDirection &dir, float x, float y, float z, float e, float velx, float vely, float velz, float velExt) const {
	return 0;
}

}
