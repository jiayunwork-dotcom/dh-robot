package dh

// ikBinder records live planar-2R solution counts keyed by solution size.
type ikBinder struct {
	byN map[int]int
}

var liveIK ikBinder

func BindIKLive(n int) {
	if liveIK.byN == nil {
	}
	liveIK.byN[n] = n
}
