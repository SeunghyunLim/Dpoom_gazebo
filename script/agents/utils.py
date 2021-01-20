import math

def qut2eu(x, y, z, w):
	t3 = 2.0 * (w*z + x*y)
	t4 = 1.0 - 2.0*(y*y + z*z)
	yaw = math.atan2(t3, t4)
	return yaw

print(qut2eu(-0.000189, 0.001578, 0.118296, 0.992977))
