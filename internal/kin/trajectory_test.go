package kin

import (
	"math"
	"testing"
)

func TestTrapezoidEndpoints(t *testing.T) {
	q, _, _ := TrapezoidProfile(10, 2, 2, 1, 0)
	if q != 0 {
		t.Fatalf("start should be 0")
	}
	qEnd, _, _ := TrapezoidProfile(10, 2, 2, 1, 2)
	if math.Abs(qEnd-10) > 1e-9 {
		t.Fatalf("end should be 10, got %v", qEnd)
	}
}

func TestCubicBlend(t *testing.T) {
	if CubicBlend(0, 10, 0) != 0 {
		t.Fatalf("frac 0")
	}
	if CubicBlend(0, 10, 1) != 10 {
		t.Fatalf("frac 1")
	}
}

func TestMultiSegmentPath(t *testing.T) {
	wp := []float64{0, 5, 10}
	got := MultiSegmentPath(wp, 0.5)
	if math.Abs(got-5) > 1e-9 {
		t.Fatalf("mid should be 5, got %v", got)
	}
}

func TestSmoothExponential(t *testing.T) {
	got := SmoothExponential(0, 10, 1, 1)
	if got <= 0 || got >= 10 {
		t.Fatalf("should move toward target, got %v", got)
	}
}
