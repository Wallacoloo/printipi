/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 

#ifndef MOTION_ACCELERATIONPROFILE_H
#define MOTION_ACCELERATIONPROFILE_H

namespace motion {

/* 
 * An AccelerationProfile takes event times and transforms them into a refined time based upon the acceleration mode.
 * As an example, a movement at 30 mm/sec with duration=0.6 sec might have Events with times like this:
 * 0.1, 0.2, 0.3, 0.4, 0.5, 0.6
 * and transform them to something like:
 * 0.2, 0.35, 0.5, 0.6, 0.75, 0.9
 * Currently, the initial and final velocity should both be 0 mm/sec.
 * Note that the events are already encoded at a *constant* velocity of Vmax (mm/sec) when they are passed through the AccelerationProfile. The AccelerationProfile should re-encode them so that the accelerate up to Vmax and then back to 0, and the velocity NEVER EXCEEDS Vmax.
 *
 * Note: AccelerationProfile is an interface and all derivatives must implement the methods outlined in the AccelerationProfile class. NoAcceleration can be considered a default implementation of this interface.
 */
struct AccelerationProfile {
	//Optional, but almost surely needed:
    inline void begin(float moveDuration, float Vmax) {
    	(void)moveDuration; (void)Vmax; //unused
    }
    //float transform(float inp, float moveDuration, float Vmax);
};

//AccelerationProfile implementation that doesn't perform any acceleration transformation
struct NoAcceleration : public AccelerationProfile {
    inline float transform(float inp) { return inp; }
};

}

#endif
