package dh

import "testing"

func TestAnalyticEndVelocityMatchesJacobian(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}, {A: 1, Alpha: 0, D: 0, Theta: 0}}}
	rates := []float64{1, 1}
	analytic := c.AnalyticEndVelocity(rates)
	jac := c.GeometricJacobian(Identity())
	fromJac := [3]float64{0, 0, 0}
	for i := 0; i < 2; i++ {
		for k := 0; k < 3; k++ {
			fromJac[k] += jac[i][k] * rates[i]
		}
	}
	for i := 0; i < 3; i++ {
		if analytic[i] != fromJac[i] {
			t.Fatalf("analytic %v != jacobian %v", analytic, fromJac)
		}
	}
}

func TestMaxLinearSpeedPositive(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}, {A: 1, Alpha: 0, D: 0, Theta: 0}}}
	if c.MaxLinearSpeed() <= 0 {
		t.Fatalf("max linear speed should be positive")
	}
}
