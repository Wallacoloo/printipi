#ifndef DRIVERS_LINEARDELTASTEPPER_H
#define DRIVERS_LINEARDELTASTEPPER_H

/* 
 * Printipi/drivers/lineardeltastepper.h
 * (c) 2014 Colin Wallace
 * 
 * LinearDeltaStepper implements the AxisStepper interface for (rail-based) Delta-style robots like the Kossel
 */

/*
	#t1,t2 were solved in Mathematica as such:
	#xyzOfT = {x ->  x0 + vx t, y ->  y0 + vy t, z ->  z0 + vz t} 
	#ABCOfT = solsABC /. xyzOfT
	#Solve[(A == A0 + s) /. ABCOfT, t]
	#FullSimplify[%]
	#Then proceed to hand-optimize
*/
/* Raspberry Pi float performance can be found here: http://www.raspberrypi.org/forums/viewtopic.php?t=7336
  float +,-,*: 2 cycles
  float /: 32 cycles (same for doubles)
  float sqrt: 48 cycles (same for doubles)
*/

#include "axisstepper.h"
#include "linearstepper.h" //for LinearHomeStepper
#include "endstop.h"

namespace drv {

template <std::size_t AxisIdx, typename CoordMap, unsigned R1000, unsigned L1000, unsigned STEPS_M, typename EndstopT=EndstopNoExist> class LinearDeltaStepper : public AxisStepper {
	private:
		float M0; //initial coordinate of THIS axis.
		int sTotal;
		//float x0, y0, z0;
		//float vx, vy, vz;
		//float v2; //squared velocity.
		float inv_v2;
		float vz_over_v2;
		float _almostTerm1; //used for caching & reducing computational complexity inside nextStep()
		float _almostRootParam;
		float _almostRootParamV2S;
		static constexpr float r() { return R1000 / 1000.; }
		static constexpr float L() { return L1000 / 1000.; }
		static constexpr float STEPS_MM() { return STEPS_M / 1000.; }
		static constexpr float MM_STEPS() { return  1. / STEPS_MM(); }
	public:
		typedef LinearHomeStepper<STEPS_M, EndstopT> HomeStepperT;
		LinearDeltaStepper() {}
		template <std::size_t sz> LinearDeltaStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
			: AxisStepper(idx, curPos, vx, vy, vz, ve),
			 M0(curPos[AxisIdx]*MM_STEPS()), 
			 sTotal(0),
			 //vx(vx), vy(vy), vz(vz),
			 //v2(vx*vx + vy*vy + vz*vz), 
			 inv_v2(1/(vx*vx + vy*vy + vz*vz)),
			 vz_over_v2(vz*inv_v2) {
			 	static_assert(AxisIdx < 3, "LinearDeltaStepper only supports axis A, B, or C (0, 1, 2)");
			 	this->time = 0; //this may NOT be zero-initialized by parent.
				float x0, y0, z0, e_;
				//CoordMap::xyzeFromMechanical(curPos, this->x0, this->y0, this->z0, e_);
				std::tie(x0, y0, z0, e_) = CoordMap::xyzeFromMechanical(curPos);
				//precompute as much as possible:
				_almostRootParamV2S = 2*M0 - 2*z0;
				if (AxisIdx == 0) {
					_almostTerm1 = inv_v2*(r()*vy - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
					//rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + (M0 + s - z0)*(M0 + s - z0));
					//rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 + 2*M0*s - 2*M0*z0 + s*s - 2*s*z0 + z0*z0);
					//rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 - 2*M0*z0 + z0*z0) - v2*s*(2*M0 + s - 2*z0);
					_almostRootParam = -inv_v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + M0*M0 - 2*M0*z0 + z0*z0);
					//_almostRootParamV2S = 2*M0 - 2*z0; // (...+s)*-1/v2*s
				} else if (AxisIdx == 1) { 
					_almostTerm1 = inv_v2*(r()*(sqrt(3)*vx - vy)/2. - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
					//rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
					_almostRootParam = -inv_v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + M0*M0 - 2*M0*z0 + z0*z0);
					//_almostRootParamV2S = 2*M0 - 2*z0;
				} else if (AxisIdx == 2) {
					_almostTerm1 = inv_v2*(-r()*(sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(M0 - z0)); // + vz/v2*s;
					//rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
					_almostRootParam = -inv_v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + M0*M0 - 2*M0*z0 + z0*z0);
					//_almostRootParamV2S = 2*M0 - 2*z0;
				}
			}
		void getTerm1AndRootParam(float &term1, float &rootParam, float s) {
			//Therefore, we should cache values calculatable at init-time, like all of the second-half on rootParam.
			term1 = _almostTerm1 + vz_over_v2*s;
			rootParam = term1*term1 + _almostRootParam - inv_v2*s*(_almostRootParamV2S + s);
			/*if (AxisIdx == 0) {
				term1 = r()*vy - vx*x0 - vy*y0 + vz*(M0 + s - z0);
				rootParam = term1*term1 - v2*(-L()*L() + x0*x0 + (r() - y0)*(r() - y0) + (M0 + s - z0)*(M0 + s - z0));
			} else if (AxisIdx == 1) { 
				term1 = r()*(sqrt(3)*vx - vy)/2. - vx*x0 - vy*y0 + vz*(M0 + s - z0);
				rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(-sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
			} else if (AxisIdx == 2) {
				term1 = -r()*(sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(M0 + s - z0);
				rootParam = term1*term1 - v2*(-L()*L() + r()*r() + x0*x0 + y0*y0 + r()*(sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
			}*/
			/*float root = std::sqrt(rootParam);
			float t1 = (term1 - root)/v2;
			float t2 = (term1 + root)/v2;*/
		}
		float testDir(float s) {
			float term1, rootParam;
			getTerm1AndRootParam(term1, rootParam, s);
			if (rootParam < 0) {
				return NAN;
			}
			//float root = std::sqrt(rootParam);
			//float t1 = (term1 - root)/v2;
			//float t2 = (term1 + root)/v2;
			//float root = std::sqrt(rootParam/v2/v2);
			//float t1 = term1/v2 - root;
			//float t2 = term1/v2 + root;
			float root = std::sqrt(rootParam);
			float t1 = term1 - root;
			float t2 = term1 + root;
			//LOGV("LinearDeltaStepper<%zu>::testDir(%f) times %f, %f\n", AxisIdx, s, t1, t2);
			if (root > term1) { //if this is true, then t1 MUST be negative.
				//return t2 if t2 > 0 else None
				//return t2 > 0 ? t2 : NAN;
				return t2 > time ? t2 : NAN;
			} else {
				//return t1;
				return t1 > time ? t1 : (t2 > time ? t2 : NAN); //ensure no value < time is returned.
			}
		}
		void _nextStep() {
			float negTime = testDir((sTotal-1)*MM_STEPS()); //get the time at which next steps would occur.
			float posTime = testDir((sTotal+1)*MM_STEPS());
			//LOGV("LinearDeltaStepper<%zu>::neg/pos/cur-time %f, %f, %f\n", AxisIdx, negTime, posTime, time);
			if (negTime < time || std::isnan(negTime)) { //negTime is invalid
				if (posTime > time) {
					//LOGV("LinearDeltaStepper<%zu>::chose %f (pos)\n", AxisIdx, posTime);
					this->time = posTime;
					this->direction = StepForward;
					++sTotal;
				} else {
					this->time = NAN;
				}
			} else if (posTime < time || std::isnan(posTime)) { //posTime is invalid
				if (negTime > time) {
					//LOGV("LinearDeltaStepper<%zu>::chose %f (neg)\n", AxisIdx, negTime);
					this->time = negTime;
					this->direction = StepBackward;
					--sTotal;
				} else {
					this->time = NAN;
				}
			} else { //neither time is invalid
				if (negTime < posTime) {
					//LOGV("LinearDeltaStepper<%zu>::chose %f (neg)\n", AxisIdx, negTime);
					this->time = negTime;
					this->direction = StepBackward;
					--sTotal;
				} else {
					//LOGV("LinearDeltaStepper<%zu>::chose %f (pos)\n", AxisIdx, posTime);
					this->time = posTime;
					this->direction = StepForward;
					++sTotal;
				}
			}
		}
};

}


#endif

