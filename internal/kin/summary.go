package kin

import (
	"fmt"
	"strings"
)

// Summary is a human-readable description of a forward-kinematics result. It
// groups the numeric outcome into sentences that a console or web page can
// print directly.
type Summary struct {
	NLinks      int
	EndPosition [3]float64
	EulerDeg    [3]float64
	ReachMin    float64
	ReachMax    float64
}

// Summarize builds a Summary from a chain spec and its forward result.
func Summarize(c ChainSpec, r Result) Summary {
	lo, hi := Reach(c.Links)
	return Summary{
		NLinks:      len(c.Links),
		EndPosition: r.EndPosition.Triple(),
		EulerDeg: [3]float64{
			rad2deg(r.EulerZYX[0]),
			rad2deg(r.EulerZYX[1]),
			rad2deg(r.EulerZYX[2]),
		},
		ReachMin: lo,
		ReachMax: hi,
	}
}

// String renders the summary as plain text.
func (s Summary) String() string {
	var b strings.Builder
	fmt.Fprintf(&b, "links: %d\n", s.NLinks)
	fmt.Fprintf(&b, "end-effector: (%.4f, %.4f, %.4f)\n",
		s.EndPosition[0], s.EndPosition[1], s.EndPosition[2])
	fmt.Fprintf(&b, "orientation (ZYX deg): yaw=%.3f pitch=%.3f roll=%.3f\n",
		s.EulerDeg[0], s.EulerDeg[1], s.EulerDeg[2])
	fmt.Fprintf(&b, "reach bound: [%.4f, %.4f]\n", s.ReachMin, s.ReachMax)
	return b.String()
}
