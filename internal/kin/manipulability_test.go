package kin

import "testing"

func TestManipulability(t *testing.T) {
	c := ChainSpec{
		Links:  []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}},
		Joints: []float64{30},
	}
	if c.Manipulability() < 0 {
		t.Fatalf("manipulability negative")
	}
	if c.ConditionNumber() < 0 {
		t.Fatalf("condition number negative")
	}
}

func TestEndVelocity(t *testing.T) {
	c := ChainSpec{
		Links:  []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}},
		Joints: []float64{30},
	}
	v := c.EndVelocity([]float64{1})
	zero := true
	for _, comp := range v {
		if comp != 0 {
			zero = false
			break
		}
	}
	if zero {
		t.Fatalf("nonzero joint rate should give nonzero end-effector velocity, got %v", v)
	}
}
