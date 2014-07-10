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
		ydiv = (2.*(4*A*A - 8*A*B + 4*B*B + 9*r*r))
		ya = 2*(A-B)*(A-B)*r
		yb = 4*Sqrt((A - B)*(A - B)*(-(A - B)*(A - B)*(A - B)*(A - B) + 4*(A - B)*(A - B)*L*L + 3*(-2*(A - B)*(A - B) + 3*L*L)*r*r - 9*r*r*r*r))
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
	za = (B - C)*r*(2*A*A*A - A*A*(B + C) - A*(B*B + C*C - 3*r*r) + (B + C)*(2*B*B - 3*B*C + 2*C*C + 3*r*r))
	zb = Sqrt(3)*Sqrt(-((B - C)*(B - C)*r*r*((A - B)*(A - B)*(A - C)*(A - C)*(B - C)*(B - C) + 3*(A*A + B*B - B*C + C*C - A*(B + C))*(A*A + B*B - B*C + C*C - A*(B + C) - 4*L*L)*r*r + 9*(2*(A*A + B*B - B*C + C*C - A*(B + C)) - 3*L*L)*r*r*r*r + 27*r*r*r*r*r*r)))
	zdiv = (B - C)*r*(4*(A*A + B*B - B*C + C*C - A*(B + C)) + 9*r*r)
	
	#z1 = (za + zb)/zdiv
	
	#z2 = (za - zb)/zdiv
	
	#will use smaller of z.
	#if sign(zb) == sign(zdiv), this should be z2, else z1.
	#therefore z = za/zdiv - abs(zb/zdiv)
	z = za/zdiv - abs(zb/zdiv)
	#Solving for x, y in terms of z gives 
	x = ((B - C)*(B + C - 2*z))/(2*Sqrt(3)*r)
	y = -((-2*A*A + B*B + C*C + 4*A*z - 2*B*z - 2*C*z)/(6*r))

	#zdiv = 0 when C == B || 
	# C == 1/2 (A + B - Sqrt[3] Sqrt[-A^2 + 2 A B - B^2 - 3 r^2]) || 
	# C == 1/2 (A + B + Sqrt[3] Sqrt[-A^2 + 2 A B - B^2 - 3 r^2])
	# only the first value of C (C == B) can be real.
	return (x, y, z)
	
def xyzsFromABCFilt(A, B, C):
	#recall that A > z, B > z and C > z.
	unfilt = xyzsFromABC(A, B, C)
	return tuple((x, y, z) for (x, y, z) in unfilt if A > z and B > z and C > z)

def relativeTimeOfNextStep(axisIdx, dir, s, x0, y0, z0, e0, vx, vy, vz, ve):
	A, B, C = ABCFromXyz(x0, y0, z0)
	v2 = (vx**2 + vy**2 + vz**2)
	term1 = r*vy - vx*x0 - vy*y0 + vz*(A + s - z0)
	rootparam = term1*term1 - v2*(-L*L + x0*x0 + (r - y0)*(r - y0) + (A + s - z0)*(A + s - z0))
	if rootparam < 0: 
		print "(wrong direction)"
		return -1
	root = Sqrt(rootparam)
	t1 = (term1 - root)/v2
	t2 = (term1 + root)/v2
	print "(times:", t1, t2, ")"
	if t1 < 0:
		return t2 #t2 may be < 0, in which case that means wrong direction.
	elif t2 < 0:
		return t1
	else:
		return min(t1, t2)
		
def relativeTimeDirOfNextStep(axisIdx, A, B, C, D, vx, vy, vz, ve):
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
		
def checkRelativeTimeOfNextStep(axisIdx, dir, s, x0, y0, z0, e0, vx, vy, vz, ve):
	args = axisIdx, dir, s, x0, y0, z0, e0, vx, vy, vz, ve
	print "relativeTimeOfNextStep", args
	t = relativeTimeOfNextStep(*args)
	if t is None: return
	print "time:", t
	ABC0 = ABCFromXyz(x0, y0, z0)
	ABC = ABCFromXyz(x0 + t*vx, y0 + t*vy, z0 + t*vz)
	print "ABCs", ABC0, ABC
	xyz0 = xyzsFromABC(*ABC0)
	xyz = xyzsFromABCFilt(*ABC)
	print "xyzs", xyz0, xyz, xyzsFromABC_(*ABC)
	
def checkRelativeTimeDirOfNextStep(axisIdx, A, B, C, D, vx, vy, vz, ve, msg=None):
	args = axisIdx, A, B, C, D, vx, vy, vz, ve
	print "relativeTimeDirOfNextStep", args
	expectdir = None
	if msg:
		if "negative" in msg.lower():
			expectdir = -1
		elif "positive" in msg.lower():
			expectdir = 1
		print msg
	t, dir = relativeTimeDirOfNextStep(*args)
	if t is None: return
	print "time, dir:", t, dir
	xyz0 = xyzsFromABC_(A, B, C)
	xyz = xyz0[0] + t*vx, xyz0[1] + t*vy, xyz0[2] + t*vz
	print "xyz0, dest:", xyz0, xyz
	if dir == expectdir:
		print "PASSED"
	else:
		print "FAILED"
	#ABC0 = [A, B, C]
	#ABC0[axisIdx] += dir
	#xyz1 = xyzsFromABC_(*ABC0)
	#print "calc xyz", xyz1
          
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
checkRelativeTimeOfNextStep(0, None, -1, 0, 0, 0, 0, 1,  0, 0, 0) #(21.886068628239236, -9.985980212144613e-17, 1.1368683772161603e-13)
checkRelativeTimeOfNextStep(0, None, -2, 0, 0, 0, 0, 1,  0, 0, 0) #(30.91924966748058, 1.172236991982472e-14, 2.2737367544323206e-13),
checkRelativeTimeOfNextStep(0, None, -3, 0, 0, 0, 0, 1,  0, 0, 0) #(37.82856063875542, -3.8354017639237094e-16, 1.4210854715202004e-13)
checkRelativeTimeOfNextStep(0, None,  3, 0, 0, 0, 0, 1,  0, 0, 0) #Correct; when moving away from x=0, A should decrease - not increase.
checkRelativeTimeOfNextStep(0, None, -2, 0, 0, 0, 0, -1, 0, 0, 0) #(-30.91924966748061, 1.2126596023639044e-14, 0.0)
checkRelativeTimeOfNextStep(0, None, -3, 0, 0, 0, 0, -1, 0, 0, 0) #(-37.82856063875544, -0.0, 0.0)
checkRelativeTimeOfNextStep(0, None,  1, 0, 0, 0, 0,  0, 1, 0, 0) #(0, 2.4346372937608436, 2.842170943040401e-14)


print "TEST RELATIVE TIME AND DIR (0)"
checkRelativeTimeDirOfNextStep(0, h, h, h, None, 1, 0, 0, None, "dir should be negative")
checkRelativeTimeDirOfNextStep(0, h, h, h, None, 4, 0, 0, None, "dir should be negative; sooner")
checkRelativeTimeDirOfNextStep(0, h, h, h, None, -1, 0, 0, None, "dir should be negative")
checkRelativeTimeDirOfNextStep(0, h, h, h, None, 0, 1, 0, None, "dir should be positive")
checkRelativeTimeDirOfNextStep(0, h, h, h, None, 0, 0, 1, None, "dir should be positive")
print "TEST RELATIVE TIME AND DIR (1)"
checkRelativeTimeDirOfNextStep(1, h, h, h, None, 1, 0, 0, None, "dir should be positive")
checkRelativeTimeDirOfNextStep(1, h, h, h, None, 4, 0, 0, None, "dir should be positive; sooner")
checkRelativeTimeDirOfNextStep(1, h, h, h, None, -1, 0, 0, None, "dir should be negative")
checkRelativeTimeDirOfNextStep(1, h, h, h, None, 0, 1, 0, None, "dir should be negative")
checkRelativeTimeDirOfNextStep(1, h, h, h, None, 0, 0, 1, None, "dir should be positive")
print "TEST RELATIVE TIME AND DIR (2)"
checkRelativeTimeDirOfNextStep(2, h, h, h, None, 1, 0, 0, None, "dir should be negative")
checkRelativeTimeDirOfNextStep(2, h, h, h, None, 4, 0, 0, None, "dir should be negative; sooner")
checkRelativeTimeDirOfNextStep(2, h, h, h, None, -1, 0, 0, None, "dir should be positive")
checkRelativeTimeDirOfNextStep(2, h, h, h, None, 0, 1, 0, None, "dir should be negative")
checkRelativeTimeDirOfNextStep(2, h, h, h, None, 0, 0, 1, None, "dir should be positive")
