package kin

import (
	"math"
	"testing"
)

func straightChain() ChainSpec {
	return ChainSpec{
		Links: []LinkSpec{
			{A: 1, Alpha: 0, D: 0, Theta: 0},
			{A: 1, Alpha: 0, D: 0, Theta: 0},
			{A: 1, Alpha: 0, D: 0, Theta: 0},
		},
		Joints: []float64{0, 0, 0},
	}
}

func TestForwardStraight(t *testing.T) {
	r, err := Forward(straightChain())
	if err != nil {
		t.Fatalf("Forward error: %v", err)
	}
	if math.Abs(r.EndPosition.X-3) > 1e-9 {
		t.Errorf("end X = %g, want 3", r.EndPosition.X)
	}
	if math.Abs(r.EndPosition.Y) > 1e-9 || math.Abs(r.EndPosition.Z) > 1e-9 {
		t.Errorf("end Y/Z must be 0, got (%g,%g)", r.EndPosition.Y, r.EndPosition.Z)
	}
}

func TestForwardSingleRotation(t *testing.T) {
	c := straightChain()
	c.Joints = []float64{90, 0, 0}
	r, err := Forward(c)
	if err != nil {
		t.Fatalf("Forward error: %v", err)
	}
	// All three links end up aligned along world +Y once the first joint
	// rotates 90° about Z, so the end-effector sits at (0, 3).
	if math.Abs(r.EndPosition.X) > 1e-9 || math.Abs(r.EndPosition.Y-3) > 1e-9 {
		t.Errorf("end = (%g,%g), want (0,3)", r.EndPosition.X, r.EndPosition.Y)
	}
}

func TestSkeletonLength(t *testing.T) {
	pts, err := Skeleton(straightChain())
	if err != nil {
		t.Fatalf("Skeleton error: %v", err)
	}
	if len(pts) < 4 { // base origin plus one point per link
		t.Errorf("skeleton points = %d, want >= 4", len(pts))
	}
}

func TestValidateRejectsTooFewJoints(t *testing.T) {
	c := straightChain()
	c.Joints = []float64{0} // fewer than 3 links
	if err := Validate(c); err == nil {
		t.Errorf("expected validation error for joint/link mismatch")
	}
}

func TestReach(t *testing.T) {
	min, max := Reach(straightChain().Links)
	if math.Abs(min) > 1e-9 {
		t.Errorf("min reach = %g, want 0", min)
	}
	if math.Abs(max-3) > 1e-9 {
		t.Errorf("max reach = %g, want 3", max)
	}
}
