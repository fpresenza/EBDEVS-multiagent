#######################################
## QSS utils
#######################################
from pypdevs.infinity import INFINITY
import numpy as np

#######################################
# minposroot(coeff, order)
# coef: array or list of coefficients
# order: int
#######################################
def minposroot(coeff, order):
	mpr = INFINITY
	## ORDER 0
	if (order == 0):
		mpr = INFINITY
	## ORDER 1
	elif(order == 1):
		if (coeff[1] == 0): ## constant polynomial
			mpr = INFINITY
		else:
			mpr = -coeff[0] / coeff[1] # if x(t) = coeff[0] + coeff[1] * t => 0 = coeff[0] + coeff[1] * t0 => t0 = -coeff[0]/coeff[1]
		## sanity check: time cannot be < 0
		if (mpr < 0):
			mpr = INFINITY
	## ORDER 2
	elif (order == 2):
		if ( coeff[2] == 0 or (1000 * abs(coeff[2])) < abs(coeff[1]) ): ## linear polynomial
			if (coeff[1] == 0): ## constant polynomial
				mpr = INFINITY
			else: ## linear polynomial
				mpr = -coeff[0] / coeff[1]
				## sanity check: time cannot be < 0
				if (mpr < 0): 
					mpr = INFINITY
		else:
			disc = coeff[1] * coeff[1] - 4 * coeff[2] * coeff[0] # discriminant
			if (disc < 0): ## no real roots
				mpr = INFINITY
			else:
				sd = np.sqrt(disc)
				r1 = (-coeff[1] + sd) / 2 / coeff[2] ## right root
				if (r1 > 0):
					mpr = r1 
				## sanity check: time cannot be < 0
				else:
					mpr = INFINITY
				r1 = (-coeff[1] - sd) / 2 / coeff[2] ## left root
				if ( (r1 > 0) and (r1 < mpr) ):
					mpr = r1

	## ORDER 3
	elif (order == 3):
		if ((coeff[3] == 0) or (1000 * abs(coeff[3]) < abs(coeff[2]))):
			mpr = minposroot(coeff, 2)
		else:
			q = (3 * coeff[3] * coeff[1] - coeff[2] * coeff[2]) / 9 / coeff[3] / coeff[3]
			r = (9 * coeff[3] * coeff[2] * coeff[1] - 27 * coeff[3] * coeff[3] * coeff[0] - 2 * coeff[2] * coeff[2] * coeff[2]) / 54 / coeff[3] / coeff[3] / coeff[3]
			disc = q * q * q + r * r # discriminant
			mpr = INFINITY
			if (disc >= 0):
				# only one real root
				sd = np.sqrt(disc)
				if (r + sd > 0): 
					s = np.power(r + sd, 1.0/3)
				else:
					s = -np.power(abs(r + sd), 1.0/3)
				if (r - sd > 0):
					t = np.power(r-sd, 1.0/3)
				else:
					t = - np.power(abs(r-sd), 1.0/3)
				r1 = s + t - coeff[2] / 3 / coeff[3]
				if (r1 > 0):
					mpr = r1
			else:
				# three real roots
				rho    = np.sqrt(-q * q * q)
				th     = np.arccos(r/rho)
				rho13  = np.power(rho, 1.0/3)
				costh3 = np.cos(th/3)
				sinth3 = np.sin(th/3)
				spt    = rho13 * 2 * costh3
				smti32 = -rho13 * sinth3 * np.sqrt(3)
				r1 = spt - coeff[2] / 3 / coeff[3]
				if (r1 > 0):
					mpr = r1
				r1 = -spt / 2 - coeff[2] / 3 / coeff[3] + smti32
				if ((r1 > 0) and (r1 < mpr)):
					mpr = r1
				r1 = r1 - 2 * smti32
				if ((r1 > 0) and (r1 < mpr)):
					mpr = r1

	# ORDER 4
	elif (order == 4):
		## Based on Ferrari's Method
		if ( (coeff[4] == 0) or (1000 * abs(coeff[4]) < abs(coeff[3]))):
			mpr = minposroot(coeff, 3)
		else:
			p = -3 * coeff[3] * coeff[3] / 8 / coeff[4] / coeff[4] + coeff[2] / coeff[4]
			q = coeff[3] * coeff[3] * coeff[3] / 8 / coeff[4] / coeff[4] / coeff[4] - \
				coeff[3] * coeff[2] / 2 / coeff[4] / coeff[4] + coeff[1] / coeff[4]
			r = -3 * np.power(coeff[3], 4) / 256 / np.power(coeff[4], 4) + \
				coeff[2] * coeff[3] * coeff[3] / 16 / np.power(coeff[4], 3) - \
				coeff[3] * coeff[1] / 4 / coeff[4] / coeff[4] + coeff[0] / coeff[4]
			co = []
			co.append(-q * q)
			co.append(p * p - 4 * r)
			co.append(2 * p)
			co.append(1)
			z0 = minposroot(co, 3)
			b1   = -np.sqrt(z0)
			c1a  = (p + z0) / 2
			c1b  = -q / 2 / b1
			db1  = coeff[3] / 2 / coeff[4]
			dc1a = coeff[3] * coeff[3] / 16 / coeff[4] / coeff[4]
			mpr  = INFINITY
			co[0] = c1a + c1b + b1/2 * db1 + dc1a
			co[1] = b1 + db1
			co[2] = 1
			r1 = minposroot(co, 2)
			if (r1 > 0):
				mpr = r1
			co[0] = c1a - c1b - b1/2 * db1 + dc1a
			co[1] = - b1 + db1
			r1 = minposroot(co, 2)
			if ((r1 > 0) and (r1 < mpr)):
				mpr = r1

	# Verification
	if(mpr >= 0): # some floting point calculations can return nan (or -nan), so checking for "mpr < 0" might not be enough
		return mpr
	else:
		return INFINITY

# def _minposroot(coeff, order):
# 	if (order == 1):
# 		return minposroot_1(coeff)
# 	elif (order == 2):
# 		return minposroot_2(coeff)
# 	elif (order == 3):
# 		return minposroot_3(coeff)
# 	else:
# 		raise ValueError('Supplied QSS order (order={}) not implemented'.format(order))

# def minposroot_1(coeff):
# 	if(coeff[1] == 0): # constant polynomial
# 		mpr = INFINITY
# 	else:
# 		mpr = -coeff[0]/coeff[1] # if x(t) = coeff[0] + coeff[1] * t => 0 = coeff[0] + coeff[1] * t0 => t0 = -coeff[0]/coeff[1]
		
# 	if(mpr < 0 or mpr >= INFINITY):
# 		mpr = INFINITY

# 	return mpr

# def minposroot_2(coeff):
# 	mpr = INFINITY
# 	return mpr

# def minposroot_3(coeff):
# 	mpr = INFINITY
# 	return mpr

# Change of temporal variable from p(t) to p(t-dt)
def advance_time(p, dt, order=None):
	# p: polynomial (list)
	# dt: elapsed time (float)
	# order: polynomial order (int)

	if (order == -1 or order == None):
		if (p[4] != 0):
			order = 4
		elif (p[3] != 0):
			order = 3
		elif (p[2] != 0):
			order = 2
		else:
			order = 1

	if(order == 1):
		p[0] = p[0] + dt * p[1]
		# p = [p[0], p[1]]
	elif (order == 2):
		p[0] = p[0] + dt * p[1] + dt * dt * p[2]
		p[1] = p[1] +  2 * p[2] * dt
		# p = [p[0], p[1], p[2]]
	elif (order == 3):
		p[0] = p[0] + dt * p[1] + dt * dt * p[2] + dt * dt * dt * p[3]
		p[1] = p[1] +  2 * p[2] * dt + 3 * p[3] * dt * dt
		p[2] = p[2] +  3 * p[3] * dt
		# p = [p[0], p[1], p[2], p[3]]
	elif (order == 4):
		p[0] = p[0] + dt * p[1] + dt * dt * p[2] + dt * dt * dt * p[3] + dt * dt * dt * dt * p[4]
		p[1] = p[1] + 2 * p[2] * dt + 3 * p[3] * dt * dt + 4 * p[4] * dt * dt * dt
		p[2] = p[2] + 3 * p[3] * dt + 6 * p[4] * dt * dt
		p[3] = p[3] + 4 * p[4] * dt
	else:
		raise ValueError('Supplied QSS order (order={}) not implemented'.format(order))
	return p

# Evaluation of the polynomial in dt
def evaluate_poly(coeff,dt,order,debug=False):
	if (order == 1):
		p = coeff[0] + \
			dt * coeff[1]
	elif (order == 2):
		p = coeff[0] + \
			dt * coeff[1] + \
			dt * dt * coeff[2]
	elif (order == 3):
		p = coeff[0] + \
			dt * coeff[1] + \
			dt * dt * coeff[2] + \
			dt * dt * dt * coeff[3]
	elif (order == 4):
		p = coeff[0] + \
			dt * coeff[1] + \
			dt * dt * coeff[2] + \
			dt * dt * dt * coeff[3] + \
			dt * dt * dt * dt * coeff[4]
	else:
		p = 0.0
	if (debug):
		print("Evaluate Poly: coeff = {}, dt = {}, order = {}, eval = {}".format(coeff,dt,order,p))
	return float(p)