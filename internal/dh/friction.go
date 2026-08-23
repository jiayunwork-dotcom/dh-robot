package dh

import "math"

// FrictionModel holds the Coulomb (static + kinetic) and viscous coefficients
// for a single joint. It converts a joint velocity and the commanded torque into
// the net torque after friction, following a simple but widely used model.
type FrictionModel struct {
	StaticCoulomb float64 // break-away torque at zero velocity
	KineticCoulomb float64 // sliding torque at any speed
	Viscous       float64 // proportional to velocity (N·m per rad/s)
}

// TorqueAt returns the friction torque opposing the motion implied by velocity v
// under a commanded torque tau. When |v| is below the stiction threshold the
// joint sticks unless tau exceeds the static limit; otherwise kinetic + viscous
// friction applies with the sign of v. The returned value is the net torque that
// actually accelerates the link.
func (f FrictionModel) TorqueAt(tau, v float64) float64 {
	const stickEps = 1e-4
	if math.Abs(v) < stickEps {
		// stick regime: friction cancels tau unless tau would break away
		if math.Abs(tau) <= f.StaticCoulomb {
			return 0
		}
		sign := 1.0
		if tau < 0 {
			sign = -1.0
		}
		return tau - sign*f.StaticCoulomb
	}
	sign := 1.0
	if v < 0 {
		sign = -1.0
	}
	return tau - sign*f.KineticCoulomb - f.Viscous*v
}

// LostWork estimates the energy dissipated by friction over a trajectory of
// sampled velocities, integrating |friction| * |velocity| * dt. The result is a
// non-negative scalar used for efficiency accounting.
func (f FrictionModel) LostWork(velocities []float64, dt float64) float64 {
	if dt <= 0 {
		dt = 1
	}
	loss := 0.0
	for _, v := range velocities {
		frictionOnly := f.KineticCoulomb + f.Viscous*math.Abs(v)
		loss += frictionOnly * math.Abs(v) * dt
	}
	return loss
}

// EquivalentViscous returns the constant that, if used as a pure viscous term,
// would dissipate the same work as the Coulomb term at the given nominal speed.
// It is handy for controllers that only support viscous damping gains.
func (f FrictionModel) EquivalentViscous(nominalSpeed float64) float64 {
	if nominalSpeed <= 0 {
		return f.KineticCoulomb
	}
	return f.KineticCoulomb / nominalSpeed
}
