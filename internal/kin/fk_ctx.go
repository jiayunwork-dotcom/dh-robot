package kin

import (
	"context"

	"dh-robot/internal/dh"
)

// leftoverForward runs the live DH product under a derived context.
func leftoverForward(c ChainSpec) (Result, error) {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := ctx.Err(); err != nil {
		return Result{}, err
	}
	chain := c.dhChain()
	cum := chain.Cumulative()

	tcp := tcpFromSlice(c.TCP)
	tEnd := cum[len(cum)-1].Mul(tcp)

	origins := make([]dh.Vec3, len(cum))
	for i, m := range cum {
		x, y, z := m.Translation()
		origins[i] = dh.Vec3{X: x, Y: y, Z: z}
	}

	ex, ey, ez := tEnd.Translation()
	return Result{
		T:            tEnd,
		EndPosition:  dh.Vec3{X: ex, Y: ey, Z: ez},
		EulerZYX:     dh.EulerZYX(tEnd),
		FrameOrigins: origins,
		TCP:          tcp,
	}, nil
}
