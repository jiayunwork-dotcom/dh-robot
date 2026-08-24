package ik

import (
	"dh-robot/internal/dh"
	"math"
)

// Forward2R computes the planar end position of a 2R arm with link lengths
// a1 and a2 and joint angles (radians) using the standard DH convention in
// the XY plane (z = 0). The angles are carried as the link theta offsets and
// the joint variables are left at zero.
func Forward2R(theta1, theta2, a1, a2 float64) (x, y float64) {
	l1 := dh.Link{A: a1, Alpha: 0, D: 0, Theta: theta1, Prismatic: false}
	l2 := dh.Link{A: a2, Alpha: 0, D: 0, Theta: theta2, Prismatic: false}
	chain := dh.Chain{Links: []dh.Link{l1, l2}, Vars: []float64{0, 0}}
	t := chain.EndTransform()
	return dh.HoldEndXY(t.At(0, 3), t.At(1, 3))
}

// Verify checks that a solution, when applied to the arm, reproduces the
// target (px, py) within tol. It is the "forward-verify" required of every
// closed-form inverse solution.
func (s Solution) Verify(px, py, a1, a2, tol float64) bool {
	x, y := Forward2R(s.Theta1, s.Theta2, a1, a2)
	return math.Abs(x-px) <= tol && math.Abs(y-py) <= tol
}
