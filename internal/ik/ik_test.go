package ik

import (
	"math"
	"testing"
)

func TestForward2RKnownPoint(t *testing.T) {
	// Elbow manipulator: a1=a2=1, theta1=0, theta2=0 -> (2,0).
	x, y := Forward2R(0, 0, 1, 1)
	if math.Abs(x-2) > 1e-9 || math.Abs(y) > 1e-9 {
		t.Errorf("Forward2R(0,0)= (%g,%g), want (2,0)", x, y)
	}
	// theta1=90, theta2=0 -> (0,2).
	x, y = Forward2R(math.Pi/2, 0, 1, 1)
	if math.Abs(x) > 1e-9 || math.Abs(y-2) > 1e-9 {
		t.Errorf("Forward2R(90,0)= (%g,%g), want (0,2)", x, y)
	}
}

func TestPlanar2RSolvesReachable(t *testing.T) {
	// Point (1,1) reachable by a 1-1 arm (distance sqrt2 < sum 2).
	px, py := 1.0, 1.0
	sols, err := Planar2R(px, py, 1, 1)
	if err != nil {
		t.Fatalf("Planar2R reachable point error: %v", err)
	}
	if len(sols) == 0 {
		t.Fatalf("expected at least one solution")
	}
	if !sols[0].Verify(px, py, 1, 1, 1e-6) {
		t.Errorf("solution does not verify")
	}
}

func TestPlanar2RUnreachable(t *testing.T) {
	// Distance > a1+a2 cannot be reached.
	_, err := Planar2R(10, 0, 1, 1)
	if err == nil {
		t.Errorf("expected error for unreachable point (dist 10 > 2)")
	}
}
