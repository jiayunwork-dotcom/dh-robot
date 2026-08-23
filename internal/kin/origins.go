package kin

import "dh-robot/internal/dh"

// FrameOrigin returns the world position of the origin of frame i. Frame 0 is
// the fixed base; frame n is the end of the last link before the tool
// transform. This is simply the translation column of the cumulative product
// A_1 * ... * A_i.
func FrameOrigin(c ChainSpec, i int) (dh.Vec3, error) {
	if err := Validate(c); err != nil {
		return dh.Vec3{}, err
	}
	chain := c.dhChain()
	cum := chain.Cumulative()
	if i < 0 || i >= len(cum) {
		return dh.Vec3{}, ErrBadFrame
	}
	x, y, z := cum[i].Translation()
	return dh.Vec3{X: x, Y: y, Z: z}, nil
}

// AxisOrigins returns every frame origin (base through end) for the chain.
// It is the same list the forward solution exposes and is handy when a caller
// only needs the polyline skeleton without the full pose.
func AxisOrigins(c ChainSpec) ([]dh.Vec3, error) {
	res, err := Forward(c)
	if err != nil {
		return nil, err
	}
	return res.FrameOrigins, nil
}
