#Info from http://www.tridprinting.com/Hot-Ends/#Thermistor
#B=3990k appears to be a typo (should be B=3990)
#Equations from http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation
B=3990.
T0 = 298.15 #25 *C
R0 = 100000.
from math import exp
R = lambda T: R0*exp(-B*(1./T0 - 1./T))
print R(298.15)
#100000.0
print R(298.15-5)
#125640.42735015175
print R(298.15+5)
#80193.75950469213
print R(298.15+100)
#3469.4405269241597
print R(298.15+1000)
#3.3334394399664875