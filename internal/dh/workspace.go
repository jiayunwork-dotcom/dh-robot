package dh

import "math"

// WorkspaceSample is one reachable point (the end-effector location) for a given
// joint configuration.
type WorkspaceSample struct {
	Joints []float64 // joint values used (radians/lengths)
	X, Y, Z float64  // end-effector position
	Reach   float64  // distance from base origin
}

// SampleWorkspace sweeps every joint over [lo, hi] with steps subdivisions and
// records the reachable end-effector positions (after the tool transform). This
// is a brute-force reachability map for open chains with few degrees of freedom.
func (c Chain) SampleWorkspace(tool Mat4, lo, hi float64, steps int) []WorkspaceSample {
	if steps < 1 {
		return nil
	}
	n := len(c.Links)
	out := make([]WorkspaceSample, 0, intPow(steps, n))
	c.recurseWorkspace(tool, lo, hi, steps, 0, make([]float64, n), &out)
	return out
}

func (c Chain) recurseWorkspace(tool Mat4, lo, hi float64, steps, idx int, cur []float64, out *[]WorkspaceSample) {
	if idx == len(c.Links) {
		// evaluate end position
		cc := Chain{Links: c.Links, Vars: append([]float64(nil), cur...)}
		x, y, z := cc.EndTransform().Mul(tool).Translation()
		*out = append(*out, WorkspaceSample{
			Joints: append([]float64(nil), cur...),
			X:      x, Y: y, Z: z,
			Reach: math.Sqrt(x*x + y*y + z*z),
		})
		return
	}
	for i := 0; i <= steps; i++ {
		v := lo + (hi-lo)*float64(i)/float64(steps)
		cur[idx] = v
		c.recurseWorkspace(tool, lo, hi, steps, idx+1, cur, out)
	}
}

func intPow(base, exp int) int {
	r := 1
	for i := 0; i < exp; i++ {
		r *= base
	}
	if r > 1000000 {
		return 1000000
	}
	return r
}

// BoundingSphere returns the minimum and maximum reach over a workspace sample,
// i.e. the inner and outer radii bounding the reachable shell.
func BoundingSphere(samples []WorkspaceSample) (inner, outer float64) {
	if len(samples) == 0 {
		return 0, 0
	}
	inner = math.MaxFloat64
	for _, s := range samples {
		if s.Reach < inner {
			inner = s.Reach
		}
		if s.Reach > outer {
			outer = s.Reach
		}
	}
	return inner, outer
}
