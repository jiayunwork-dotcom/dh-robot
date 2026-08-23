package dh

import "testing"

func TestInertiaAt(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	if c.InertiaAt(2) <= 0 {
		t.Fatalf("inertia should be positive")
	}
}

func TestGravityTorqueZeroMass(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}, {A: 1, Alpha: 0, D: 0, Theta: 0}}}
	tq := c.GravityTorque(0, 9.81) // zero mass -> zero torque
	for _, v := range tq {
		if v != 0 {
			t.Fatalf("gravity torque with m=0 should be 0, got %v", tq)
		}
	}
}

func TestKineticEnergyNonNegative(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	ke := c.KineticEnergy(1, []float64{1})
	if ke < 0 {
		t.Fatalf("kinetic energy negative: %v", ke)
	}
}

func TestMonteCarloReachFraction(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}, {A: 1, Alpha: 0, D: 0, Theta: 0}}}
	frac, rMin, rMax := c.MonteCarloReach(200, 5, 42)
	if frac < 0 || frac > 1 {
		t.Fatalf("fraction out of range: %v", frac)
	}
	if rMax <= 0 {
		t.Fatalf("rMax should be positive")
	}
	if rMin <= 0 {
		t.Fatalf("rMin should be positive for reachable samples")
	}
}
