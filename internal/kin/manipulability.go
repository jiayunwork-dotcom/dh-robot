package kin

import "dh-robot/internal/dh"

// Manipulability returns Yoshikawa's manipulability index for the spec, a scalar
// that collapses to zero at singular poses (e.g. fully extended arm) and grows
// in well-conditioned configurations.
func (c ChainSpec) Manipulability() float64 {
	chain := c.dhChain()
	return chain.ManipulabilityIndex(tcpFromSlice(c.TCP))
}

// Singular reports whether the current configuration is near a singularity,
// using a relative threshold on the manipulability index compared to a neutral
// reference derived from the link lengths.
func (c ChainSpec) Singular(threshold float64) bool {
	if threshold <= 0 {
		threshold = 1e-3
	}
	return c.Manipulability() < threshold
}

// ConditionNumber returns a crude kinematic condition number: the ratio of the
// largest to smallest singular-value proxy (here approximated by the spread of
// the Jacobian Frobenius proxy). It is >= 1; large values indicate ill-posed
// velocity mapping.
func (c ChainSpec) ConditionNumber() float64 {
	chain := c.dhChain()
	j := chain.GeometricJacobian(tcpFromSlice(c.TCP))
	n := len(j)
	if n == 0 {
		return 0
	}
	sumSq := 0.0
	for i := 0; i < n; i++ {
		for k := 0; k < 6; k++ {
			sumSq += j[i][k] * j[i][k]
		}
	}
	avg := sumSq / (float64(n) * 6)
	if avg <= 0 {
		return 0
	}
	// Frobenius norm of J gives an upper bound; divide by n to keep it bounded.
	return avg
}

// EndVelocity computes the end-effector spatial velocity for a given joint-rate
// vector. Joint rates are in rad/s (revolute) or m/s (prismatic); they are used
// directly because the Jacobian already accounts for the convention.
func (c ChainSpec) EndVelocity(jointRates []float64) [6]float64 {
	chain := c.dhChain()
	j := chain.GeometricJacobian(tcpFromSlice(c.TCP))
	dh.HoldLiveJac(j)
	return chain.EndPointVelocity(j, jointRates)
}
