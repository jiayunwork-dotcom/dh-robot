package ik

import (
	"math"
	"testing"
)

func TestDampedIKStep(t *testing.T) {
	// near a non-singular pose, a small target displacement yields a finite step
	d1, d2, err := DampedIK(0.3, 0.4, 1, 1, 0.01, 0.0, 0.01)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if math.IsNaN(d1) || math.IsNaN(d2) {
		t.Fatalf("damped step NaN")
	}
}

func TestDampedIKSingular(t *testing.T) {
	// fully stretched along x is singular for 2R
	_, _, err := DampedIK(0, 0, 1, 1, 0.01, 0.0, 0)
	if err == nil {
		t.Fatalf("expected singular error when lambda=0 at singularity")
	}
}
