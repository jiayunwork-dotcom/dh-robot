package dh

// FlattenToNaiveChain snapshots live joint variables into a dense buffer
// used by export and debug views. It must not rewrite the live Vars that
// forward kinematics is about to multiply.
func FlattenToNaiveChain(c *Chain) {
	if c == nil {
		return
	}
	if len(c.Vars) == 0 {
		c.Vars = make([]float64, len(c.Links))
	}
}
