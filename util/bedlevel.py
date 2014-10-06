import numpy as np
import rotation_matrix as r
np.set_printoptions(suppress=True) #No scientific notation
orig = np.array([0, 0, 1.]) #perpendicular to bed
#meas = np.array([.01, 0, .99]) #real perp to bed.
#meas = np.array([.002, 0, .99])
#meas = np.array([0.002, 0.0018, .99])
#meas = np.array([0.000, 0.0018, .99])
#meas = np.array([-0.0012, 0.0018, .99])
#meas = np.array([-0.0014, 0.0014, .99])
#meas = np.array([-0.0018, 0.0014, .99])
#meas = np.array([-0.0022, 0.0010, .99])
#meas = np.array([-.003, 0.0010, 0.99])
#meas = np.array([-.004, 0.0009, 0.99])
meas = np.array([.01, 0.00, .99])

def CPP(matr):
	pieces = [[], [], []]
	for (a, b), val in np.ndenumerate(matr):
		pieces[a].append(int(val*1000000000))
	body = ", \n".join("%i, %i, %i" %tuple(p) for p in pieces)
	return "<%s, 1000000000>" %body

def I3():
	return np.matrix([[1., 0, 0], [0, 1., 0], [0, 0, 1.]])
m = I3()
r.R_2vect(m, orig, meas)


print orig, "->", meas
print m
print CPP(m)
