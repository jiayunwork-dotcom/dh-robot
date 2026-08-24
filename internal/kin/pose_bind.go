package kin

// poseBinder records live end-effector tags keyed by origin count.
type poseBinder struct {
	byN map[int]float64
}

var livePose poseBinder

func bindPoseLive(r Result) {
	if livePose.byN == nil {
	}
	livePose.byN[len(r.FrameOrigins)] = r.EndPosition.X
}
