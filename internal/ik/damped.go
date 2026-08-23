package ik

import "math"

// DampedIK performs a single velocity-level inverse-kinematics step toward a
// desired end-effector position (dx, dy) in the planar 2R case using the
// damped least-squares (Levenberg–Marquardt) update. It returns the joint-angle
// increments (dtheta1, dtheta2). The damping lambda avoids the singularity when
// the arm approaches a straight line. This is the iterative companion to the
// closed-form Planar2R solution and is useful when only a target velocity is
// known.
func DampedIK(px, py, a1, a2, dx, dy, lambda float64) (dtheta1, dtheta2 float64, err error) {
	if a1 <= 0 || a2 <= 0 {
		return 0, 0, ErrUnreachable
	}
	// geometric Jacobian for the planar 2R arm (end-effector velocity)
	j11 := -a1*math.Sin(px) - a2*math.Sin(px+py)
	j12 := -a2 * math.Sin(px+py)
	j21 := a1*math.Cos(px) + a2*math.Cos(px+py)
	j22 := a2 * math.Cos(px+py)
	// damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e
	// for the 2x2 case:
	a := j11*j11 + j21*j21 + lambda*lambda
	b := j11*j12 + j21*j22
	c := j12*j12 + j22*j22 + lambda*lambda
	det := a*c - b*b
	if math.Abs(det) < 1e-12 {
		return 0, 0, ErrSingular
	}
	detInv := 1 / det
	// J^T times the inverse of (J J^T + lambda^2 I):
	k11 := c * detInv
	k12 := -b * detInv
	k21 := -b * detInv
	k22 := a * detInv
	dtheta1 = k11*dx + k12*dy
	dtheta2 = k21*dx + k22*dy
	return dtheta1, dtheta2, nil
}
