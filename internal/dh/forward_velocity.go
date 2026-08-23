package dh

import "math"

// AnalyticEndVelocity computes the end-effector linear velocity for a single
// revolute joint chain directly from the kinematic chain, without building the
// full Jacobian matrix. It is a lightweight alternative to GeometricJacobian for
// the common case of a single motion query. qdot is the joint-rate vector. It
// returns (vx, vy, vz) in the base frame.
func (c Chain) AnalyticEndVelocity(qdot []float64) [3]float64 {
	cum := c.Cumulative()
	n := len(c.Links)
	ex, ey, ez := cum[n].Translation()
	end := [3]float64{ex, ey, ez}
	vel := [3]float64{0, 0, 0}
	for i := 0; i < n; i++ {
		j := c.Links[i]
		if j.Prismatic {
			// linear motion along the joint axis of frame i
			zx, zy, zz := cum[i][2], cum[i][6], cum[i][10]
			rate := 0.0
			if i < len(qdot) {
				rate = qdot[i]
			}
			vel[0] += zx * rate
			vel[1] += zy * rate
			vel[2] += zz * rate
			continue
		}
		// revolute: omega = z_{i-1} x r, r = p_end - p_{i-1}
		zx, zy, zz := cum[i][2], cum[i][6], cum[i][10]
		px, py, pz := cum[i].Translation()
		rx := zy*(end[2]-pz) - zz*(end[1]-py)
		ry := zz*(end[0]-px) - zx*(end[2]-pz)
		rz := zx*(end[1]-py) - zy*(end[0]-px)
		rate := 0.0
		if i < len(qdot) {
			rate = qdot[i]
		}
		vel[0] += rx * rate
		vel[1] += ry * rate
		vel[2] += rz * rate
	}
	return vel
}

// MaxLinearSpeed returns the largest linear speed across all joints moving at the
// given uniform unit rate (1 rad/s or 1 m/s). It is a quick worst-case estimate
// for speed limiting and is always non-negative.
func (c Chain) MaxLinearSpeed() float64 {
	cum := c.Cumulative()
	n := len(c.Links)
	ex, ey, ez := cum[n].Translation()
	end := [3]float64{ex, ey, ez}
	best := 0.0
	for i := 0; i < n; i++ {
		px, py, pz := cum[i].Translation()
		rx := cum[i][6]*(end[2]-pz) - cum[i][10]*(end[1]-py)
		ry := cum[i][10]*(end[0]-px) - cum[i][2]*(end[2]-pz)
		rz := cum[i][2]*(end[1]-py) - cum[i][6]*(end[0]-px)
		s := math.Sqrt(rx*rx + ry*ry + rz*rz)
		if s > best {
			best = s
		}
	}
	return best
}
