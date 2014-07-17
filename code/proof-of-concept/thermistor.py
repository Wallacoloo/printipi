#Info from http://www.tridprinting.com/Hot-Ends/#Thermistor
#B=3990k appears to be a typo (should be B=3990)
#Equations from http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation
#For TridPrinting thermistor:
B=3990.
T0 = 298.15 #25 *C
R0 = 100000.
#For RioRand thermistor:
#B=3950.
#T0 = 298.15 #25 *C
#R0 = 100000.
from math import exp, log
#R = R0 e^[-B(1/T0-1/T)]
R = lambda T: R0*exp(-B*(1./T0 - 1./T))
#ln(R/R0)/-B = 1/T0 - 1/T
#1/T = 1/T0 + ln(R/R0)/B
T = lambda R: 1. / (1./T0 + log(R/R0)/B)
print "R 25C:", R(298.15)
#100000.0
print "T:", T(100000.0)
#293.15
print "R 20C:", R(298.15-5)
#125640.42735015175
print "T:", T(125640.42735015175)
#303.15
print "R 30C:", R(298.15+5)
#80193.75950469213
print "T:", T(80193.75950469213)
print "R 125C:", R(298.15+100)
#3469.4405269241597
print "T:", T(3469.4405269241597)
#398.15
print "R 1025C:", R(298.15+1000)
#3.3334394399664875
print "T:", T(3.3334394399664875)
#1298.15
