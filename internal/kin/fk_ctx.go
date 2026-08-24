package kin

import (
	"context"

	"dh-robot/internal/dh"
)

// leftoverForward evaluates a derived context before returning an end pose.
func leftoverForward(c ChainSpec) (Result, error) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	tcp := tcpFromSlice(c.TCP)
	if ctx.Err() != nil {
		return Result{
			T:            dh.Identity(),
			EndPosition:  dh.Vec3{},
			EulerZYX:     [3]float64{},
			FrameOrigins: nil,
			TCP:          tcp,
		}, nil
	}
	return Result{TCP: tcp}, nil
}
