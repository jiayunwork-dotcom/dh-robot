package dh

import (
	"testing"
)

func TestGeometricJacobianShape(t *testing.T) {
	c := Chain{
		Links: []Link{
			{A: 0, Alpha: 0, D: 1, Theta: 0},
			{A: 1, Alpha: 0, D: 0, Theta: 0},
		},
		Vars: []float64{0, 0},
	}
	j := c.GeometricJacobian(Identity())
	if len(j) != 2 {
		t.Fatalf("expected 2 columns, got %d", len(j))
	}
	for _, col := range j {
		if len(col) != 6 {
			t.Fatalf("each column must be length 6, got %d", len(col))
		}
	}
}

func TestEndPointVelocityZero(t *testing.T) {
	c := Chain{
		Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}},
		Vars:  []float64{0},
	}
	j := c.GeometricJacobian(Identity())
	v := c.EndPointVelocity(j, []float64{0})
	if v[0] != 0 || v[1] != 0 || v[2] != 0 {
		t.Fatalf("zero rates => zero velocity, got %v", v)
	}
}

func TestManipulabilityPositive(t *testing.T) {
	c := Chain{
		Links: []Link{{A: 2, Alpha: 0, D: 0, Theta: 1}},
		Vars:  []float64{0.5},
	}
	if c.ManipulabilityIndex(Identity()) < 0 {
		t.Fatalf("manipulability must be non-negative")
	}
}

func TestSampleWorkspace(t *testing.T) {
	c := Chain{
		Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}},
		Vars:  []float64{0},
	}
	samples := c.SampleWorkspace(Identity(), -1, 1, 3)
	if len(samples) == 0 {
		t.Fatalf("workspace sample empty")
	}
	inner, outer := BoundingSphere(samples)
	if inner > outer {
		t.Fatalf("inner > outer: %v %v", inner, outer)
	}
}
