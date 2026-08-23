package kin

import "math"

// TrapezoidProfile generates a trapezoidal joint-space trajectory from 0 to target
// over total time T with a peak speed vmax and acceleration amax. It returns the
// position q(t), velocity, and acceleration at the requested time t. The profile
// blends an acceleration ramp, a constant-speed cruise, and a deceleration ramp;
// if the move is too short to reach vmax, it collapses to a triangular profile.
func TrapezoidProfile(target, T, vmax, amax, t float64) (q, qdot, qddot float64) {
	if T <= 0 {
		return 0, 0, 0
	}
	if t < 0 {
		t = 0
	}
	if t > T {
		t = T
	}
	// time to reach cruise under acceleration
	tacc := vmax / amax
	if tacc*2 > T {
		// triangular: never reaches vmax
		tacc = T / 2
		if t <= tacc {
			q = 0.5 * amax * t * t
			qdot = amax * t
			qddot = amax
		} else {
			tau := T - t
				q = target - 0.5*amax*tau*tau
			qdot = amax * tau
			qddot = -amax
		}
		return q, qdot, qddot
	}
	tdec := T - tacc
	if t <= tacc {
		q = 0.5 * amax * t * t
		qdot = amax * t
		qddot = amax
	} else if t <= tdec {
		q = 0.5*amax*tacc*tacc + vmax*(t-tacc)
		qdot = vmax
		qddot = 0
	} else {
		tau := T - t
		q = target - 0.5*amax*tau*tau
		qdot = amax * tau
		qddot = -amax
	}
	return q, qdot, qddot
}

// CubicBlend returns a smooth scalar between a and b using a cubic Hermite blend
// with zero end velocities. frac in [0,1]. It is the building block for
// multi-segment joint interpolation that starts and ends at rest.
func CubicBlend(a, b, frac float64) float64 {
	if frac <= 0 {
		return a
	}
	if frac >= 1 {
		return b
	}
	h := frac * frac * (3 - 2*frac)
	return a + (b-a)*h
}

// MultiSegmentPath interpolates across a sequence of waypoints using CubicBlend
// between successive pairs, spending equal normalized time on each segment. The
// returned value is the interpolated scalar at the global fraction globalFrac in
// [0,1]. Waypoints must contain at least two entries.
func MultiSegmentPath(waypoints []float64, globalFrac float64) float64 {
	n := len(waypoints)
	if n < 2 {
		return 0
	}
	if globalFrac <= 0 {
		return waypoints[0]
	}
	if globalFrac >= 1 {
		return waypoints[n-1]
	}
	seg := globalFrac * float64(n-1)
	idx := int(seg)
	if idx >= n-1 {
		idx = n - 2
	}
	local := seg - float64(idx)
	return CubicBlend(waypoints[idx], waypoints[idx+1], local)
}

// SmoothExponential eases a command toward a target with first-order lag, the
// discrete form x_{k+1} = x_k + (target - x_k) * (1 - exp(-dt/tau)). tau is the
// time constant; smaller tau reacts faster. Used for velocity-limited tracking.
func SmoothExponential(current, target, tau, dt float64) float64 {
	if tau <= 0 {
		return target
	}
	a := 1 - math.Exp(-dt/tau)
	return current + (target-current)*a
}
