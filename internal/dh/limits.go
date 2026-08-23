package dh

import "math"

// JointLimit pairs a minimum and maximum value for one joint angle (radians) or
// displacement (metres for prismatic joints). A nil limit means "unbounded".
type JointLimit struct {
	Min float64
	Max float64
}

// Within reports whether the given joint value sits inside the limit. A limit is
// considered satisfied when the value is within [Min, Max] inclusive, with a
// small tolerance for floating-point edges.
func (jl JointLimit) Within(v float64) bool {
	const eps = 1e-9
	return v >= jl.Min-eps && v <= jl.Max+eps
}

// ClampTo brings a joint value inside the limit without exceeding it, returning
// the original value when inside and the nearest bound when outside.
func (jl JointLimit) ClampTo(v float64) float64 {
	if v < jl.Min {
		return jl.Min
	}
	if v > jl.Max {
		return jl.Max
	}
	return v
}

// JointLimits is an ordered set of per-joint limits, one per link.
type JointLimits []JointLimit

// AllWithin reports whether the whole configuration respects every limit.
func (js JointLimits) AllWithin(values []float64) bool {
	for i, lim := range js {
		if i >= len(values) {
			break
		}
		if !lim.Within(values[i]) {
			return false
		}
	}
	return true
}

// ClampConfig returns a new slice with every out-of-range joint clamped to its
// bound; values beyond the limit list are passed through unchanged.
func (js JointLimits) ClampConfig(values []float64) []float64 {
	out := make([]float64, len(values))
	copy(out, values)
	for i, lim := range js {
		if i < len(out) {
			out[i] = lim.ClampTo(out[i])
		}
	}
	return out
}

// ReachMargin returns the smallest normalized slack across all joints, i.e. the
// minimum of (value-distance-to-bound)/range. A value of 0 means some joint sits
// exactly on a limit; negative means out of range. Used to warn before slamming
// a stop.
func (js JointLimits) ReachMargin(values []float64) float64 {
	worst := math.MaxFloat64
	for i, lim := range js {
		if i >= len(values) {
			break
		}
		span := lim.Max - lim.Min
		if span <= 0 {
			continue
		}
		dMin := (values[i] - lim.Min) / span
		dMax := (lim.Max - values[i]) / span
		slack := dMin
		if dMax < slack {
			slack = dMax
		}
		if slack < worst {
			worst = slack
		}
	}
	if worst == math.MaxFloat64 {
		return 0
	}
	return worst
}

// SelfCollision is a crude proximity check between consecutive link segments in
// the plane. It treats each link as a line segment from frame i-1 to frame i and
// flags an overlap when the perpendicular distance between non-adjacent segments
// drops below a clearance. Returns true when any forbidden pair is too close.
func (c Chain) SelfCollision(clearance float64) bool {
	cum := c.Cumulative()
	n := len(c.Links)
	for i := 0; i < n; i++ {
		for j := i + 2; j < n; j++ {
			ax, ay, _ := cum[i].Translation()
			bx, by, _ := cum[i+1].Translation()
			cx, cy, _ := cum[j].Translation()
			dx, dy, _ := cum[j+1].Translation()
			if segDist(ax, ay, bx, by, cx, cy, dx, dy) < clearance {
				return true
			}
		}
	}
	return false
}

func segDist(ax, ay, bx, by, cx, cy, dx, dy float64) float64 {
	// distance between segment AB and segment CD (projected to 2D)
	d1 := pointSegDist(cx, cy, ax, ay, bx, by)
	d2 := pointSegDist(dx, dy, ax, ay, bx, by)
	d3 := pointSegDist(ax, ay, cx, cy, dx, dy)
	d4 := pointSegDist(bx, by, cx, cy, dx, dy)
	return math.Min(math.Min(d1, d2), math.Min(d3, d4))
}

func pointSegDist(px, py, ax, ay, bx, by float64) float64 {
	dx := bx - ax
	dy := by - ay
	len := dx*dx + dy*dy
	var t float64
	if len > 0 {
		t = ((px-ax)*dx + (py-ay)*dy) / len
		if t < 0 {
			t = 0
		}
		if t > 1 {
			t = 1
		}
	}
	qx := ax + t*dx
	qy := ay + t*dy
	return math.Sqrt((px-qx)*(px-qx) + (py-qy)*(py-qy))
}
