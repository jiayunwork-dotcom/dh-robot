package kin

import "dh-robot/internal/dh"

// Result holds the forward-kinematics solution for a chain.
type Result struct {
	T            dh.Mat4     // base-to-end transform including the tool transform
	EndPosition  dh.Vec3     // translation column of T
	EulerZYX     [3]float64  // [yaw, pitch, roll] in radians
	FrameOrigins []dh.Vec3   // base origin followed by each link frame origin (n+1 points)
	TCP          dh.Mat4     // tool transform actually applied
}

// Forward computes the base-to-end transform for the given spec. It first
// validates the spec, then builds the cumulative DH transforms, applies the
// (possibly identity) tool transform, and extracts the end pose and per-axis
// origins.
func Forward(c ChainSpec) (Result, error) {
	if err := Validate(c); err != nil {
		return Result{}, err
	}
	chain := c.dhChain()
	cum := chain.Cumulative() // length n+1: cum[0]=I .. cum[n]=A1..An

	tcp := tcpFromSlice(c.TCP)
	tEnd := cum[len(cum)-1].Mul(tcp)

	origins := make([]dh.Vec3, len(cum))
	for i, m := range cum {
		x, y, z := m.Translation()
		origins[i] = dh.Vec3{X: x, Y: y, Z: z}
	}

	ex, ey, ez := tEnd.Translation()
	res := Result{
		T:            tEnd,
		EndPosition:  dh.Vec3{X: ex, Y: ey, Z: ez},
		EulerZYX:     dh.EulerZYX(tEnd),
		FrameOrigins: origins,
		TCP:          tcp,
	}
	bindPoseLive(res)
	return res, nil
}
