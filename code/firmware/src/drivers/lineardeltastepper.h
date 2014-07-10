#ifndef DRIVERS_LINEARDELTASTEPPER_H
#define DRIVERS_LINEARDELTASTEPPER_H

#include "axisstepper.h"

/*
	#t1,t2 were solved in Mathematica as such:
	#xyzOfT = {x ->  x0 + vx t, y ->  y0 + vy t, z ->  z0 + vz t} 
	#ABCOfT = solsABC /. xyzOfT
	#Solve[(A == A0 + s) /. ABCOfT, t]
	#FullSimplify[%]
	#Then proceed to hand-optimize
	x0, y0, z0 = xyzsFromABC_(A, B, C)
	def testDir(s):
		v2 = (vx**2 + vy**2 + vz**2)
		if axisIdx == 0:
			#t1 = (r*vy + A0*vz + s*vz - vx*x0 - vy*y0 - Sqrt(4*(r*vy - vx*x0 - vy*y0 + vz*(A0 + s - z0))**2 - 4*(vx**2 + vy**2 + vz**2)*(-L**2 + x0**2 + (r - y0)**2 + (A0 + s - z0)**2))/2. - vz*z0)/(vx**2 + vy**2 + vz**2)
			#t2 = (r*vy + A0*vz + s*vz - vx*x0 - vy*y0 + Sqrt(4*(r*vy - vx*x0 - vy*y0 + vz*(A0 + s - z0))**2 - 4*(vx**2 + vy**2 + vz**2)*(-L**2 + x0**2 + (r - y0)**2 + (A0 + s - z0)**2))/2. - vz*z0)/(vx**2 + vy**2 + vz**2)
			#term1 = r*vy + A*vz + s*vz - vx*x0 - vy*y0 - vz*z0
			#rootparam = (r*vy - vx*x0 - vy*y0 + vz*(A + s - z0))**2 - v2*(-L**2 + x0**2 + (r - y0)**2 + (A + s - z0)**2)
			term1 = r*vy - vx*x0 - vy*y0 + vz*(A + s - z0)
			rootparam = term1*term1 - v2*(-L*L + x0*x0 + (r - y0)*(r - y0) + (A + s - z0)*(A + s - z0))
		elif axisIdx == 1:
			#t1 = -(-(Sqrt(3)*r*vx) + r*vy - 2*B0*vz - 2*s*vz + 2*vx*x0 + 2*vy*y0 + Sqrt((-(Sqrt(3)*r*vx) + r*vy + 2*vx*x0 + 2*vy*y0 - 2*vz*(B0 + s - z0))**2 - 4*(vx**2 + vy**2 + vz**2)*(-L**2 + r**2 + x0**2 + y0**2 + r*(-(Sqrt(3)*x0) + y0) + (B0 + s - z0)**2)) + 2*vz*z0)/(2.*(vx**2 + vy**2 + vz**2))
			#t2 = (Sqrt(3)*r*vx - r*vy + 2*B0*vz + 2*s*vz - 2*vx*x0 - 2*vy*y0 + Sqrt((-(Sqrt(3)*r*vx) + r*vy + 2*vx*x0 + 2*vy*y0 - 2*vz*(B0 + s - z0))**2 - 4*(vx**2 + vy**2 + vz**2)*(-L**2 + r**2 + x0**2 + y0**2 + r*(-(Sqrt(3)*x0) + y0) + (B0 + s - z0)**2)) - 2*vz*z0)/(2.*(vx**2 + vy**2 + vz**2))
			#term1 = 0.5*(Sqrt(3)*r*vx - r*vy + 2*B0*vz + 2*s*vz - 2*vx*x0 - 2*vy*y0 - 2*vz*z0)
			#rootparam = 0.25*((-Sqrt(3)*r*vx + r*vy + 2*vx*x0 + 2*vy*y0 - 2*vz*(B + s - z0))**2 - 4*v2*(-L**2 + r**2 + x0**2 + y0**2 + r*(-Sqrt(3)*x0 + y0) + (B + s - z0)**2))
			term1 = (r*(Sqrt(3)*vx - vy))/2. - vx*x0 - vy*y0 + vz*(B + s - z0)
			rootparam = term1*term1 - v2*(-L*L + r*r + x0*x0 + y0*y0 + r*(-Sqrt(3)*x0 + y0) + (B + s - z0)*(B + s - z0))
		elif axisIdx == 2:
			#t1 = -(r*(Sqrt(3)*vx + vy) + Sqrt((Sqrt(3)*r*vx + r*vy + 2*vx*x0 + 2*vy*y0 - 2*vz*(C0 + s - z0))**2 - 4*(vx**2 + vy**2 + vz**2)*(-L**2 + r**2 + x0**2 + y0**2 + r*(Sqrt(3)*x0 + y0) + (C0 + s - z0)**2)) + 2*(vx*x0 + vy*y0 - vz*(C0 + s - z0)))/(2.*(vx**2 + vy**2 + vz**2))
			#t2 = (-(r*(Sqrt(3)*vx + vy)) + Sqrt((Sqrt(3)*r*vx + r*vy + 2*vx*x0 + 2*vy*y0 - 2*vz*(C0 + s - z0))**2 - 4*(vx**2 + vy**2 + vz**2)*(-L**2 + r**2 + x0**2 + y0**2 + r*(Sqrt(3)*x0 + y0) + (C0 + s - z0)**2)) - 2*(vx*x0 + vy*y0 - vz*(C0 + s - z0)))/(2.*(vx**2 + vy**2 + vz**2))
			#term1 = -r*(Sqrt(3)*vx + vy)/2 - (vx*x0 + vy*y0 - vz*(C + s - z0))
			#rootparam = 0.25*(Sqrt(3)*r*vx + r*vy + 2*vx*x0 + 2*vy*y0 - 2*vz*(C + s - z0))**2 - v2*(-L**2 + r**2 + x0**2 + y0**2 + r*(Sqrt(3)*x0 + y0) + (C + s - z0)**2)
			term1 = -r*(Sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(C + s - z0)
			rootparam = term1*term1 - v2*(-L*L + r*r + x0*x0 + y0*y0 + r*(Sqrt(3)*x0 + y0) + (C + s - z0)*(C + s - z0))
			#t1 = (term1 - root)/(v2)
			#t2 = (term1 + root)/(v2)
		if rootparam < 0:
			print "(times: None)" 
			return None
		root = sqrt(rootparam)
		t1 = (term1 - root)/v2
		t2 = (term1 + root)/v2
		print "(times:", t1, t2, ")"
		if root > term1:
			return t2 if t2 > 0 else None
		else:
			return t1
	neg = testDir(-1), -1
	pos = testDir(1), 1
	#Return the smallest non-negative term:
	filt = [a for a in (neg, pos) if a[0] is not None and a[0] >= 0]
	if len(filt) == 0:
		return None, None
	elif len(filt) == 1:
		return filt[0]
	else:
		return filt[0] if filt[0][0] < filt[0][1] else filt[1]
*/

#define A0LOGV(format, args...) \
	if (AxisIdx==0) { LOGV(format, ## args); }

namespace drv {

template <std::size_t AxisIdx, typename CoordMath, unsigned R1000, unsigned L1000, unsigned STEPS_M> class LinearDeltaStepper : public AxisStepper {
	private:
		float M0; //initial coordinate of THIS axis.
		int sTotal;
		float x0, y0, z0;
		float vx, vy, vz;
		float v2; //squared velocity.
		static constexpr float r = R1000 / 1000.;
		static constexpr float L = L1000 / 1000.;
		static constexpr float STEPS_MM = STEPS_M / 1000.;
		static constexpr float MM_STEPS = 1. / STEPS_MM;
	public:
		LinearDeltaStepper() {}
		template <std::size_t sz> LinearDeltaStepper(int idx, const std::array<int, sz>& curPos, float vx, float vy, float vz, float ve)
			: AxisStepper(idx, curPos, vx, vy, vz, ve),
			 M0(curPos[AxisIdx]*MM_STEPS), 
			 sTotal(0),
			 vx(vx), vy(vy), vz(vz),
			 v2(vx*vx + vy*vy + vz*vz) {
				float e_;
				CoordMath::xyzeFromMechanical(curPos, this->x0, this->y0, this->z0, e_);
			}
		void getTerm1AndRootParam(float &term1, float &rootParam, float s) {
			if (AxisIdx == 0) {
				term1 = r*vy - vx*x0 - vy*y0 + vz*(M0 + s - z0);
				rootParam = term1*term1 - v2*(-L*L + x0*x0 + (r - y0)*(r - y0) + (M0 + s - z0)*(M0 + s - z0));
			} else if (AxisIdx == 1) { 
				term1 = r*(sqrt(3)*vx - vy)/2. - vx*x0 - vy*y0 + vz*(M0 + s - z0);
				rootParam = term1*term1 - v2*(-L*L + r*r + x0*x0 + y0*y0 + r*(-sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
			} else if (AxisIdx == 2) {
				term1 = -r*(sqrt(3)*vx + vy)/2 - vx*x0 - vy*y0 + vz*(M0 + s - z0);
				rootParam = term1*term1 - v2*(-L*L + r*r + x0*x0 + y0*y0 + r*(sqrt(3)*x0 + y0) + (M0 + s - z0)*(M0 + s - z0));
				//t1 = (term1 - root)/(v2)
				//t2 = (term1 + root)/(v2)
			}
		}
		float testDir(float s) {
			float term1, rootParam;
			getTerm1AndRootParam(term1, rootParam, s);
			if (rootParam < 0) {
				return NAN;
			}
			float root = std::sqrt(rootParam);
			float t1 = (term1 - root)/v2;
			float t2 = (term1 + root)/v2;
			A0LOGV("LinearDeltaStepper<%u>::testDir(%f) times %f, %f\n", AxisIdx, s, t1, t2);
			if (root > term1) { //t1 MUST be negative.
				//return t2 if t2 > 0 else None
				return t2 > 0 ? t2 : NAN;
			} else {
				return t1;
			}
		}
		void _nextStep() {
			float negTime = testDir((sTotal-1)*MM_STEPS); //get the time at which next steps would occur.
			float posTime = testDir((sTotal+1)*MM_STEPS);
			A0LOGV("LinearDeltaStepper<%u>::neg/pos/cur-time %f, %f\n", AxisIdx, negTime, posTime, time);
			if (negTime < time || std::isnan(negTime)) { //negTime is invalid
				if (posTime > time) {
					A0LOGV("LinearDeltaStepper<%lu>::chose %f (pos)\n", AxisIdx, posTime);
					this->time = posTime;
					this->direction = StepForward;
					++sTotal;
				} else {
					this->time = NAN;
				}
			} else if (posTime < time || std::isnan(posTime)) { //posTime is invalid
				if (negTime > time) {
					A0LOGV("LinearDeltaStepper<%u>::chose %f (neg)\n", AxisIdx, negTime);
					this->time = negTime;
					this->direction = StepBackward;
					--sTotal;
				} else {
					this->time = NAN;
				}
			} else { //neither time is invalid
				if (negTime < posTime) {
					A0LOGV("LinearDeltaStepper<%u>::chose %f (neg)\n", AxisIdx, negTime);
					this->time = negTime;
					this->direction = StepBackward;
					--sTotal;
				} else {
					A0LOGV("LinearDeltaStepper<%u>::chose %f (pos)\n", AxisIdx, posTime);
					this->time = posTime;
					this->direction = StepForward;
					++sTotal;
				}
			}
		}
};

}


#endif

