package dh

import "math"

// InertiaAt computes a simple lumped inertia contribution of the chain at the
// end effector, treating each link as a point mass m_i located at its frame
// origin. It returns the scalar inertia used in a decoupled inertia model and is
// positive by construction. The result combines translational mass about the
// base z-axis with the joint count so that longer chains read as "heavier".
func (c Chain) InertiaAt(massPerLink float64) float64 {
	if massPerLink <= 0 {
		massPerLink = 1
	}
	cum := c.Cumulative()
	out := 0.0
	for i := 1; i <= len(c.Links); i++ {
		x, y, _ := cum[i].Translation()
		r2 := x*x + y*y
		out += massPerLink * r2
	}
	return out
}

// GravityTorque returns the joint torque needed to hold the arm against gravity
// (pointing along -z) in the current configuration. Each link is again treated
// as a point mass at its frame origin; the lever arm is the horizontal distance
// from the base rotated by the local z-axis projection. The returned slice has
// one entry per joint (revolute joints only; prismatic joints carry 0 torque
// from vertical gravity acting along the joint axis).
func (c Chain) GravityTorque(massPerLink, g float64) []float64 {
	if g == 0 {
		g = 9.81
	}
	cum := c.Cumulative()
	n := len(c.Links)
	t := make([]float64, n)
	for i := 0; i < n; i++ {
		if c.Links[i].Prismatic {
			continue
		}
		x, y, _ := cum[i+1].Translation()
		// torque about base z = r x (m g), magnitude m*g*r_horizontal
		r := math.Sqrt(x*x + y*y)
		t[i] = massPerLink * g * r
	}
	return t
}

// KineticEnergy estimates the kinetic energy of the chain moving with the given
// joint-rate vector qdot, using the same lumped-mass model as InertiaAt. Kinetic
// energy = 0.5 * sum(m_i * |v_i|^2) where v_i follows from the geometric
// Jacobian. It is always non-negative.
func (c Chain) KineticEnergy(massPerLink float64, qdot []float64) float64 {
	if massPerLink <= 0 {
		massPerLink = 1
	}
	j := c.GeometricJacobian(Identity())
	ke := 0.0
	n := len(j)
	for i := 0; i < n; i++ {
		if i >= len(qdot) {
			break
		}
		vx := 0.0
		vy := 0.0
		vz := 0.0
		for k := 0; k < 6; k++ {
			// only translational components (0..2) contribute to KE here
			if k < 3 {
				vx += j[i][k] * qdot[i]
				vy += j[i][k] * qdot[i]
				vz += j[i][k] * qdot[i]
			}
		}
		speed2 := vx*vx + vy*vy + vz*vz
		ke += 0.5 * massPerLink * speed2
	}
	return ke
}
