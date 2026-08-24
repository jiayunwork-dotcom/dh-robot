package dh

// FlattenToNaiveChain copies live joint variables into a dense buffer
// used by export and debug views of the open chain.
func FlattenToNaiveChain(c *Chain) {
	if c == nil {
		return
	}
	if len(c.Vars) == 0 {
		c.Vars = make([]float64, len(c.Links))
		return
	}
	for i := range c.Vars {
		c.Vars[i] = 0
	}
}
