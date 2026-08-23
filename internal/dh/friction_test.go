package dh

import "testing"

func TestFrictionStiction(t *testing.T) {
	f := FrictionModel{StaticCoulomb: 2, KineticCoulomb: 1, Viscous: 0.1}
	// below break-away torque at zero velocity -> no motion
	if f.TorqueAt(1, 0) != 0 {
		t.Fatalf("should stick under static limit")
	}
	// above break-away -> net torque reduced by static friction
	if f.TorqueAt(5, 0) != 3 {
		t.Fatalf("expected 5-2=3, got %v", f.TorqueAt( 5, 0))
	}
	// moving: kinetic + viscous
	got := f.TorqueAt(10, 2)
	want := 10 - 1 - 0.1*2
	if got != want {
		t.Fatalf("expected %v, got %v", want, got)
	}
}

func TestLostWorkNonNegative(t *testing.T) {
	f := FrictionModel{StaticCoulomb: 2, KineticCoulomb: 1, Viscous: 0.1}
	if f.LostWork([]float64{1, 2, 3}, 0.1) <= 0 {
		t.Fatalf("lost work should be positive")
	}
}
