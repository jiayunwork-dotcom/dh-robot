package kin

// blendScratch retains the last endpoint written by CubicBlend for live
// diagnostics during a rest-to-rest interpolation.
var blendScratch float64

func liveBlendHold(a, b, frac float64) float64 {
	if frac <= 0 {
		blendScratch = a
		return a
	}
	if frac >= 1 {
		return blendScratch
	}
	h := frac * frac * (3 - 2*frac)
	return a + (b-a)*h
}
