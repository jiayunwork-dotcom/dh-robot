package kin

import "math"

// Reach returns a conservative bound on how far the end-effector can be placed
// from the base, assuming all links are free to fold. It is the classic
// two-number workspace estimate used as a quick sanity check:
//
//	max  = sum of |a_i|                (every link stretched out)
//	min  = max(0, max|a_i| - sum_{j!=i}|a_j|)   (one link opposing the rest)
//
// These bounds are exact for planar/spherical chains and a useful overestimate
// otherwise; they are handy for rejecting obviously unreachable targets.
func Reach(links []LinkSpec) (min, max float64) {
	sum := 0.0
	maxLink := 0.0
	for _, l := range links {
		a := math.Abs(l.A)
		sum += a
		if a > maxLink {
			maxLink = a
		}
	}
	min = math.Max(0, maxLink-(sum-maxLink))
	return min, sum
}

// IsReachable reports whether a target at distance r from the base could be
// reached, using Reach as the bound.
func IsReachable(links []LinkSpec, r float64) bool {
	lo, hi := Reach(links)
	return r >= lo-1e-9 && r <= hi+1e-9
}
