from math import sqrt
Power = pow
Sqrt = sqrt

#d=200. #distance between towers
r = 100. #distance from tower to center of build area.
L=260. #length of rods
h=800. #height of towers

def ABCFromXyz(x, y, z):
	A = Sqrt(L**2 - r**2 - x**2 + 2*r*y - y**2) + z
	B = Sqrt(L**2 - r**2 + Sqrt(3)*r*x - x**2 - r*y - y**2) + z
	C = Sqrt(L**2 - r**2 - Sqrt(3)*r*x - x**2 - r*y - y**2) + z
	return A, B, C

def xyzsFromABC(A, B, C):
	if A == B == C: #handle a division-by-zero.
		return (0, 0, A-sqrt(L*L-r*r))
	elif B == C: #handle a division-by-zero.
		y1 = (2*A**2*r - 4*A*B*r + 2*B**2*r - Sqrt((-2*A**2*r + 4*A*B*r - 2*B**2*r)**2 - 4*(4*A**2 - 8*A*B + 4*B**2 + 9*r**2)*(A**4 - 4*A**3*B + 6*A**2*B**2 - 4*A*B**3 + B**4 - 4*A**2*L**2 + 8*A*B*L**2 - 4*B**2*L**2 + 4*A**2*r**2 - 8*A*B*r**2 + 4*B**2*r**2)))/(2.*(4*A**2 - 8*A*B + 4*B**2 + 9*r**2))
		y2 = (2*A**2*r - 4*A*B*r + 2*B**2*r + Sqrt((-2*A**2*r + 4*A*B*r - 2*B**2*r)**2 - 4*(4*A**2 - 8*A*B + 4*B**2 + 9*r**2)*(A**4 - 4*A**3*B + 6*A**2*B**2 - 4*A*B**3 + B**4 - 4*A**2*L**2 + 8*A*B*L**2 - 4*B**2*L**2 + 4*A**2*r**2 - 8*A*B*r**2 + 4*B**2*r**2)))/(2.*(4*A**2 - 8*A*B + 4*B**2 + 9*r**2))
		z1 = (A**2 - B**2 - 3*r*y1)/(2.*(A - B))
		z2 = (A**2 - B**2 - 3*r*y2)/(2.*(A - B))
		return (0, y1, z1), (0, y2, z2)
	x1 = (2*Sqrt(3)*B**3*C*r - 2*Sqrt(3)*B*C**3*r + 4*Sqrt(3)*A**3*(-B + C)*r + 6*Sqrt(3)*A**2*(B**2 - C**2)*r + 3*Sqrt(3)*B**2*r**3 - 3*Sqrt(3)*C**2*r**3 - 2*Sqrt(3)*A*(B - C)*r*(B**2 + 4*B*C + C**2 + 3*r**2) - 6*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))))/(6.*r**2*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))

	y1 = (6*A**2*B**3*r - 6*A*B**4*r - 18*A**2*B**2*C*r + 12*A*B**3*C*r + 6*B**4*C*r + 18*A**2*B*C**2*r - 18*B**3*C**2*r - 6*A**2*C**3*r - 12*A*B*C**3*r + 18*B**2*C**3*r + 6*A*C**4*r - 6*B*C**4*r + 6*A**2*B*r**3 - 6*A*B**2*r**3 - 3*B**3*r**3 - 6*A**2*C*r**3 + 15*B**2*C*r**3 + 6*A*C**2*r**3 - 15*B*C**2*r**3 + 3*C**3*r**3 - 4*Sqrt(3)*A*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))) + 2*Sqrt(3)*B*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))) + 2*Sqrt(3)*C*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))))/(6.*(B - C)*r**2*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))

	z1 = (2*B**4*r + 2*A**3*(B - C)*r - 3*B**3*C*r + 3*B*C**3*r - 2*C**4*r + A**2*(-B**2 + C**2)*r + 3*B**2*r**3 - 3*C**2*r**3 - A*(B - C)*r*(B**2 + C**2 - 3*r**2) + Sqrt(3)*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))))/((B - C)*r*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))

	x2 = (2*Sqrt(3)*B**3*C*r - 2*Sqrt(3)*B*C**3*r + 4*Sqrt(3)*A**3*(-B + C)*r + 6*Sqrt(3)*A**2*(B**2 - C**2)*r + 3*Sqrt(3)*B**2*r**3 - 3*Sqrt(3)*C**2*r**3 - 2*Sqrt(3)*A*(B - C)*r*(B**2 + 4*B*C + C**2 + 3*r**2) + 6*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))))/(6.*r**2*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))

	y2 = (6*A**2*B**3*r - 6*A*B**4*r - 18*A**2*B**2*C*r + 12*A*B**3*C*r + 6*B**4*C*r + 18*A**2*B*C**2*r - 18*B**3*C**2*r - 6*A**2*C**3*r - 12*A*B*C**3*r + 18*B**2*C**3*r + 6*A*C**4*r - 6*B*C**4*r + 6*A**2*B*r**3 - 6*A*B**2*r**3 - 3*B**3*r**3 - 6*A**2*C*r**3 + 15*B**2*C*r**3 + 6*A*C**2*r**3 - 15*B*C**2*r**3 + 3*C**3*r**3 + 4*Sqrt(3)*A*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))) - 2*Sqrt(3)*B*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))) - 2*Sqrt(3)*C*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))))/(6.*(B - C)*r**2*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))

	z2 = (2*B**4*r + 2*A**3*(B - C)*r - 3*B**3*C*r + 3*B*C**3*r - 2*C**4*r + A**2*(-B**2 + C**2)*r + 3*B**2*r**3 - 3*C**2*r**3 - A*(B - C)*r*(B**2 + C**2 - 3*r**2) - Sqrt(3)*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2))))))/((B - C)*r*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))
	return (x1, y1, z1), (x2, y2, z2)
	

def xyzsFromABC_(A, B, C):
	if A == B == C: #handle a division-by-zero.
		return (0, 0, A-sqrt(L*L-r*r))
	elif B == C: #handle a division-by-zero.
		ya = 2*A**2*r - 4*A*B*r + 2*B**2*r
		yb = Sqrt((-2*A**2*r + 4*A*B*r - 2*B**2*r)**2 - 4*(4*A**2 - 8*A*B + 4*B**2 + 9*r**2)*(A**4 - 4*A**3*B + 6*A**2*B**2 - 4*A*B**3 + B**4 - 4*A**2*L**2 + 8*A*B*L**2 - 4*B**2*L**2 + 4*A**2*r**2 - 8*A*B*r**2 + 4*B**2*r**2))
		ydiv = (2.*(4*A**2 - 8*A*B + 4*B**2 + 9*r**2))
		#y1 = (ya - yb)/ydiv
		#y2 = (ya + yb)/ydiv
		#z1 = (A**2 - B**2 - 3*r*y1)/(2.*(A - B))
		#z2 = (A**2 - B**2 - 3*r*y2)/(2.*(A - B))
		#y = (z*2*(A-B) - A**2 + B**2) / -3r
		#z = (A**2 - B**2 - 3*r*(ya +/- yb)/ydiv)/(2.*(A - B))
		#z = (A-B)(A+B)/2/(A-B) - 3*r*(ya +/- yb)/ydiv/2/(A-B)
		#z = (A+B)/2 - 3/2*r*(ya +/- yb)/ydiv/(A-B)
		#z = (A+B)/2 - 3/2*r*(ya/ydiv/(A-B) +/- yb/ydiv/(A-B))
		#zmin = (A+B)/2 - 3/2*r*(ya/ydiv/(A-B) + abs(yb/ydiv/(A-B)))
		com1 = abs(yb/((A-B)*ydiv))
		com2 = ya/ydiv
		z = 0.5*(A+B - 3*r*(com2/(A-B) + com1))
		y = com2 + (A-B)*com1
		#y = (ya + (A-B)*ydiv*com1)/ydiv
		#y = (z*2*(A-B) - A**2 + B**2) / -3/r
		#y = (((A+B)/2 - 3./2*r*(ya/ydiv/(A-B) + abs(yb/ydiv/(A-B))))*2*(A-B) - A**2 + B**2) / -3/r
		#y = (ya + (A*ydiv - B*ydiv)*abs(yb/((A-B)*ydiv)))/ydiv
		##z must be < A and < B
		##A may or may not be > B
		##if A > B, then (A^2-B^2) = (A-B)(A+B) > 0, else < 0
		
		#A MUST be > B, and y MUST be < A
		#Therefore want the largest y to get the smalles z.
		#y = ya/ydiv +/- yb/ydiv
		#Largest y will be ya/ydiv + abs(yb/ydiv)
		#y = ya/ydiv + abs(yb/ydiv)
		#z = (A**2 - B**2 - 3*r*y)/(2.*(A - B))
		return (0, y, z)
		#return (0, y1, z1), (0, y2, z2)
	za = 2*B**4*r + 2*A**3*(B - C)*r - 3*B**3*C*r + 3*B*C**3*r - 2*C**4*r + A**2*(-B**2 + C**2)*r + 3*B**2*r**3 - 3*C**2*r**3 - A*(B - C)*r*(B**2 + C**2 - 3*r**2)
	zb = Sqrt(3)*Sqrt(-((B - C)**2*r**2*(B**4*(C**2 + 3*r**2) + A**4*(B**2 - 2*B*C + C**2 + 3*r**2) - 2*A**3*(B + C)*(B**2 - 2*B*C + C**2 + 3*r**2) - 6*B*C*r**2*(C**2 - 2*L**2 + 3*r**2) - 2*B**3*(C**3 + 3*C*r**2) + B**2*(C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4) + 3*r**2*(C**4 - 9*L**2*r**2 + 9*r**4 + C**2*(-4*L**2 + 6*r**2)) + A**2*(B**4 + 2*B**3*C + 2*B*C**3 + C**4 + 9*C**2*r**2 - 12*L**2*r**2 + 18*r**4 + B**2*(-6*C**2 + 9*r**2)) - 2*A*(B + C)*(B**3*C + B**2*(-2*C**2 + 3*r**2) + 3*r**2*(C**2 - 2*L**2 + 3*r**2) + B*(C**3 - 3*C*r**2)))))
	zdiv = ((B - C)*r*(4*A**2 + 4*B**2 - 4*B*C + 4*C**2 - 4*A*(B + C) + 9*r**2))
	
	#z1 = (za + zb)/zdiv
	
	#z2 = (za - zb)/zdiv
	
	#will use smaller of z.
	#if sign(zb) == sign(zdiv), this should be z2, else z1.
	#therefore z = za/zdiv - abs(zb/zdiv)
	z = za/zdiv - abs(zb/zdiv)
	#Solving for x, y in terms of z gives 
	x = ((B - C)*(B + C - 2*z))/(2*Sqrt(3)*r)
	y = -((-2*A**2 + B**2 + C**2 + 4*A*z - 2*B*z - 2*C*z)/(6*r))

	#zdiv = 0 when C == B || 
	# C == 1/2 (A + B - Sqrt[3] Sqrt[-A^2 + 2 A B - B^2 - 3 r^2]) || 
	# C == 1/2 (A + B + Sqrt[3] Sqrt[-A^2 + 2 A B - B^2 - 3 r^2])
	# only the first value of C (C == B) can be real.
	return (x, y, z),
	
def xyzsFromABCFilt(A, B, C):
	#recall that A > z, B > z and C > z.
	unfilt = xyzsFromABC(A, B, C)
	return tuple((x, y, z) for (x, y, z) in unfilt if A > z and B > z and C > z)

def relativeTimeOfNextStep(axisIdx, dir, s, x0, y0, z0, e0, vx, vy, vz, ve):
	A, B, C = ABCFromXyz(x0, y0, z0)
	v2 = (vx**2 + vy**2 + vz**2)
	term1 = r*vy + A*vz + s*vz - vx*x0 - vy*y0 - vz*z0
	rootparam = 4*(r*vy + A*vz + s*vz - vx*x0 - vy*y0 - vz*z0)**2 - 4*v2*(A**2 - L**2 + r**2 + s**2 + x0**2 - 2*r*y0 + y0**2 + 2*A*(s - z0) - 2*s*z0 + z0**2)
	if rootparam < 0: 
		print "(wrong direction)"
		return -1
	root = Sqrt(rootparam)
	t1 = (term1 - root/2.)/v2
	t2 = (term1 + root/2.)/v2
	print "(times:", t1, t2, ")"
	if t1 < 0:
		return t2 #t2 may be < 0, in which case that means wrong direction.
	elif t2 < 0:
		return t1
	else:
		return min(t1, t2)
		
def checkRelativeTimeOfNextStep(axisIdx, dir, s, x0, y0, z0, e0, vx, vy, vz, ve):
	args = axisIdx, dir, s, x0, y0, z0, e0, vx, vy, vz, ve
	print "relativeTimeOfNextStep", args
	t = relativeTimeOfNextStep(*args)
	print "time:", t
	ABC0 = ABCFromXyz(x0, y0, z0)
	ABC = ABCFromXyz(x0 + t*vx, y0 + t*vy, z0 + t*vz)
	print "ABCs", ABC0, ABC
	xyz0 = xyzsFromABC(*ABC0)
	xyz = xyzsFromABCFilt(*ABC)
	print "xyzs", xyz0, xyz, xyzsFromABC_(*ABC)
          
print xyzsFromABC(h, h-1, h-2)
print xyzsFromABC_(h, h-1, h-2) #Should be positive x, more positive y
print xyzsFromABC(h, h, h) #should be (0, 0, *)
print xyzsFromABC_(h, h, h) #should be (0, 0, *)
print xyzsFromABC(h, h-1, h-1) #should be (0, +, *)
print xyzsFromABC_(h, h-1, h-1) #should be (0, +, *)
print xyzsFromABC(h-2, h-1, h-1) #should be (0, -, *)
print xyzsFromABC_(h-2, h-1, h-1) #should be (0, -, *)
print xyzsFromABC(h-50, h-10, h-30) #should be (+, -, *)
print xyzsFromABC_(h-50, h-10, h-30) #should be (+, -, *)
checkRelativeTimeOfNextStep(0, None, -1, 0, 0, 0, 0, 1,  0, 0, 0) #Correct[xyzs 1];
checkRelativeTimeOfNextStep(0, None, -2, 0, 0, 0, 0, 1,  0, 0, 0) #Correct[xyzs 1];
checkRelativeTimeOfNextStep(0, None, -3, 0, 0, 0, 0, 1,  0, 0, 0) #Correct[xyzs 1];
checkRelativeTimeOfNextStep(0, None,  3, 0, 0, 0, 0, 1,  0, 0, 0) #Correct; when moving away from x=0, A should decrease - not increase.
checkRelativeTimeOfNextStep(0, None, -2, 0, 0, 0, 0, -1, 0, 0, 0) #Correct[xyzs 0];
checkRelativeTimeOfNextStep(0, None, -3, 0, 0, 0, 0, -1, 0, 0, 0) #Correct[xyzs 0]; even though the z-value is negative, it's a VERY small magnitude.
checkRelativeTimeOfNextStep(0, None,  1, 0, 0, 0, 0,  0, 1, 0, 0) #Correct[xyzs 1]; WRONG auto.
