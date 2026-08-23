package kin

import "dh-robot/internal/dh"

// Skeleton returns the polyline through the frame origins (the base, each
// link origin, and finally the tool-attached end point). The last vertex is
// the end-effector origin after applying the tool transform, so the drawn
// chain reflects the tool offset whenever one is supplied. The points always
// come from the actual forward-kinematics computation - the visualization
// layer must never hard-code them.
func Skeleton(c ChainSpec) ([]dh.Vec3, error) {
	res, err := Forward(c)
	if err != nil {
		return nil, err
	}
	pts := make([]dh.Vec3, 0, len(res.FrameOrigins)+1)
	pts = append(pts, res.FrameOrigins...)
	ex, ey, ez := res.T.Translation()
	pts = append(pts, dh.Vec3{X: ex, Y: ey, Z: ez})
	return pts, nil
}
