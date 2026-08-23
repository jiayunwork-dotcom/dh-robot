package dh

import (
	"math"
	"testing"
)

func TestSlerpEndpoints(t *testing.T) {
	q0 := [4]float64{1, 0, 0, 0}
	q1 := [4]float64{0, 1, 0, 0}
	a := Slerp(q0, q1, 0)
	b := Slerp(q0, q1, 1)
	if a[0] != 1 || b[1] != 1 {
		t.Fatalf("endpoints wrong: %v %v", a, b)
	}
}

func TestSlerpMidpoint(t *testing.T) {
	q0 := [4]float64{1, 0, 0, 0}
	q1 := [4]float64{0, 1, 0, 0}
	m := Slerp(q0, q1, 0.5)
	n := math.Sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2] + m[3]*m[3])
	if math.Abs(n-1) > 1e-9 {
		t.Fatalf("slerp midpoint not unit: %v", m)
	}
}

func TestAngleBetweenQuats(t *testing.T) {
	q0 := [4]float64{1, 0, 0, 0}
	q1 := [4]float64{1, 0, 0, 0}
	if AngleBetweenQuats(q0, q1) != 0 {
		t.Fatalf("identical quats should give angle 0")
	}
}
