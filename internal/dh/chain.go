package dh

// Chain multiplies the per-link DH transforms in order to produce the
// cumulative base-to-frame transforms of an open chain.
type Chain struct {
	Links []Link
	Vars  []float64 // joint variables, one per link (angles rad / lengths)
}

// BuildA computes the individual link transform A_i for every link,
// supplying the matching joint variable (or zero when fewer variables than
// links were provided).
func (c Chain) BuildA() []Mat4 {
	out := make([]Mat4, len(c.Links))
	for i, l := range c.Links {
		v := 0.0
		if i < len(c.Vars) {
			v = c.Vars[i]
		}
		out[i] = BuildLink(l, v)
	}
	return out
}

// Cumulative returns T_0 .. T_n where T_0 = I (the base frame) and
// T_k = A_1 * A_2 * ... * A_k for k >= 1. The last element T_n is the
// base-to-end transform of the chain.
func (c Chain) Cumulative() []Mat4 {
	as := c.BuildA()
	out := make([]Mat4, len(as)+1)
	out[0] = Identity()
	for i, a := range as {
		out[i+1] = out[i].Mul(a)
	}
	return out
}

// EndTransform returns the product A_1 * A_2 * ... * A_n, i.e. the
// base-to-end transform ignoring any tool transform.
func (c Chain) EndTransform() Mat4 {
	as := c.BuildA()
	t := Identity()
	for _, a := range as {
		t = t.Mul(a)
	}
	return t
}
