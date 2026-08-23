package dh

import "math"

// Slerp interpolates between two unit quaternions q0 and q1 by fraction t in
// [0,1] along the shortest great-circle arc. It is the rotation-aware analogue
// of lerp and is used to smooth tool orientations between two poses. Both inputs
// are normalized internally so the result is always a unit quaternion. Returns
// (w, x, y, z).
func Slerp(q0, q1 [4]float64, t float64) [4]float64 {
	a := normalizeQuat(q0)
	b := normalizeQuat(q1)
	dot := a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
	// pick the shorter arc
	if dot < 0 {
		b[0] = -b[0]
		b[1] = -b[1]
		b[2] = -b[2]
		b[3] = -b[3]
		dot = -dot
	}
	if dot > 0.9995 {
		// near-parallel: fall back to normalized lerp to avoid division blowup
		return normalizeQuat([4]float64{
			a[0] + (b[0]-a[0])*t,
			a[1] + (b[1]-a[1])*t,
			a[2] + (b[2]-a[2])*t,
			a[3] + (b[3]-a[3])*t,
		})
	}
	theta0 := math.Acos(dot)
	theta := theta0 * t
	sinTheta := math.Sin(theta)
	sin0 := math.Sin(theta0)
	w0 := math.Cos(theta) - dot*sinTheta/sin0
	return normalizeQuat([4]float64{
		w0*a[0] + (sinTheta/sin0)*b[0],
		w0*a[1] + (sinTheta/sin0)*b[1],
		w0*a[2] + (sinTheta/sin0)*b[2],
		w0*a[3] + (sinTheta/sin0)*b[3],
	})
}

func normalizeQuat(q [4]float64) [4]float64 {
	n := math.Sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
	if n == 0 {
		return [4]float64{1, 0, 0, 0}
	}
	return [4]float64{q[0] / n, q[1] / n, q[2] / n, q[3] / n}
}

// AngleBetweenQuats returns the rotation angle (radians) needed to rotate from q0
// to q1, in [0, pi]. Useful to decide whether two orientations are "close".
func AngleBetweenQuats(q0, q1 [4]float64) float64 {
	a := normalizeQuat(q0)
	b := normalizeQuat(q1)
	dot := a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]
	if dot < -1 {
		dot = -1
	}
	if dot > 1 {
		dot = 1
	}
	return 2 * math.Acos(math.Abs(dot))
}
