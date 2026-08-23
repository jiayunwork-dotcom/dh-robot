package dh

import (
	"math"
	"math/rand"
)

// MonteCarloReach estimates the reachable volume fraction of the workspace by
// sampling random joint configurations and counting how many place the
// end-effector within a bounding sphere of radius bound centred at the origin.
// It returns the fraction (0..1) plus the smallest and largest reachable radius
// observed, which together bound the usable workspace without an analytic map.
func (c Chain) MonteCarloReach(samples int, bound float64, seed int64) (fraction, rMin, rMax float64) {
	if samples <= 0 {
		samples = 1000
	}
	rng := rand.New(rand.NewSource(seed))
	cum := c.Cumulative()
	inside := 0
	rMin = math.MaxFloat64
	rMax = 0.0
	for s := 0; s < samples; s++ {
		for i := range c.Links {
			c.Links[i].Theta = (rng.Float64()*2 - 1) * math.Pi
		}
		cc := c.Cumulative()
		x, y, z := cc[len(cc)-1].Translation()
		r := math.Sqrt(x*x + y*y + z*z)
		if r <= bound {
			inside++
		}
		if r < rMin {
			rMin = r
		}
		if r > rMax {
			rMax = r
		}
	}
	_ = cum // baseline cumulative retained for reference of the home pose
	if inside == 0 {
		rMin = 0
	}
	return float64(inside) / float64(samples), rMin, rMax
}

// ReachabilityIndex returns a scalar 0..1 estimating how central the home pose is
// within the sampled workspace: it is the ratio of the home radius to the
// maximum sampled radius. A value near 1 means the home pose sits at the far
// edge of the workspace (poor), near 0 means deep inside (good margin).
func (c Chain) ReachabilityIndex(samples int, seed int64) float64 {
	_, _, rMax := c.MonteCarloReach(samples, 1e9, seed)
	x, y, z := c.Cumulative()[len(c.Links)].Translation()
	rHome := math.Sqrt(x*x + y*y + z*z)
	if rMax <= 0 {
		return 0
	}
	return rHome / rMax
}
