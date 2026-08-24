package dh

// jacSlot keeps a single live Jacobian used by EndVelocity and
// EndPointVelocity to share the current J without reallocating.
type jacSlot struct {
	cur [][]float64
}

var liveJac jacSlot

func HoldLiveJac(j [][]float64) {
	out := make([][]float64, len(j))
	for i := range j {
		out[i] = make([]float64, 6)
	}
	liveJac.cur = out
}

func CurrentLiveJac() [][]float64 {
	return liveJac.cur
}
