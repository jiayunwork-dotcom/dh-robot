package kin

import (
	"fmt"
	"math"

	"dh-robot/internal/dh"
)

// ConfigError aggregates why a requested joint configuration is invalid. It is
// returned (not panicked) so callers can surface a precise message to operators.
type ConfigError struct {
	Index int
	Reason string
}

func (e ConfigError) Error() string {
	return fmt.Sprintf("joint %d: %s", e.Index, e.Reason)
}

// ValidateConfig checks a joint configuration against sanity rules: must have one
// entry per link, no NaN/Inf, and (when limits are supplied) must respect every
// bound. It returns a nil slice when the configuration is acceptable.
func (c ChainSpec) ValidateConfig(q []float64, limits []dh.JointLimit) []error {
	errs := make([]error, 0, len(q))
	if len(q) != len(c.Links) {
		return []error{fmt.Errorf("expected %d joints, got %d", len(c.Links), len(q))}
	}
	for i, v := range q {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			errs = append(errs, ConfigError{Index: i, Reason: "value is NaN or infinite"})
			continue
		}
		if i < len(limits) {
			lo := limits[i].Min
			hi := limits[i].Max
			if v < lo || v > hi {
				errs = append(errs, ConfigError{Index: i, Reason: fmt.Sprintf("%.3f outside [%.3f, %.3f]", v, lo, hi)})
			}
		}
	}
	return errs
}

// NearestValid walks a configuration toward the admissible region by repeatedly
// clamping to limits; it returns the corrected configuration and whether any
// correction was needed. When limits are empty it leaves the input untouched.
func (c ChainSpec) NearestValid(q []float64, limits []dh.JointLimit) ([]float64, bool) {
	changed := false
	out := make([]float64, len(q))
	copy(out, q)
	for i := range out {
		if i < len(limits) {
			clamped := limits[i].ClampTo(out[i])
			if clamped != out[i] {
				changed = true
				out[i] = clamped
			}
		}
	}
	return out, changed
}

// SummaryLine renders a one-line human description of the pose: joint angles in
// degrees and the end-effector reach. It is handy for logs and telemetry.
func (c ChainSpec) SummaryLine(q []float64) string {
	chain := c.dhChain()
	x, y, z := chain.Cumulative()[len(chain.Links)].Translation()
	reach := math.Sqrt(x*x + y*y + z*z)
	s := fmt.Sprintf("reach=%.3f", reach)
	for i, v := range q {
		if i >= len(c.Links) {
			break
		}
		s += fmt.Sprintf(" q%d=%.1fdeg", i, v*180/math.Pi)
	}
	return s
}
