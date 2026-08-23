package dh

import (
	"math"
	"testing"
)

func TestJointLimitWithin(t *testing.T) {
	lim := JointLimit{Min: -1, Max: 1}
	if !lim.Within(0) {
		t.Fatalf("0 should be within [-1,1]")
	}
	if lim.Within(2) {
		t.Fatalf("2 should be outside")
	}
}

func TestClampConfig(t *testing.T) {
	lims := JointLimits{{Min: -1, Max: 1}, {Min: 0, Max: 2}}
	out := lims.ClampConfig([]float64{5, -3})
	if out[0] != 1 || out[1] != 0 {
		t.Fatalf("clamp failed: %v", out)
	}
}

func TestReachMargin(t *testing.T) {
	lims := JointLimits{{Min: -1, Max: 1}, {Min: 0, Max: 2}}
	m := lims.ReachMargin([]float64{0, 1})
	if m <= 0 || m > 1 {
		t.Fatalf("margin should be in (0,1], got %v", m)
	}
}

func TestSelfCollisionNoOverlap(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}, {A: 1, Alpha: 0, D: 0, Theta: 3.14}}}
	if c.SelfCollision(0.01) {
		t.Fatalf("open chain should not self-collide")
	}
}

func TestPointSegDist(t *testing.T) {
	d := pointSegDist(0, 2, -1, 0, 1, 0)
	if math.Abs(d-2) > 1e-9 {
		t.Fatalf("expected 2, got %v", d)
	}
}
