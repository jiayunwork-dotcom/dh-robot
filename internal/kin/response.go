package kin

import "dh-robot/internal/dh"

// FKResponse is the JSON body returned by POST /api/fk. It carries the full
// base-to-end homogeneous transform, the end-effector position, the ZYX
// Euler orientation (radians), and the list of axis origins. On failure the
// Error field is populated and the other fields are zero.
type FKResponse struct {
	NLinks      int           `json:"n_links"`
	T           [4][4]float64 `json:"t"`
	EndPosition [3]float64    `json:"end_position"`
	EulerZYX    [3]float64    `json:"euler_zyx"` // radians: yaw, pitch, roll
	AxisOrigins [][3]float64  `json:"axis_origins"`
	Error       string        `json:"error,omitempty"`
}

// SkeletonResponse is the JSON body returned by POST /api/skeleton. Points is
// the polyline through the frame origins and the end-effector.
type SkeletonResponse struct {
	Points [][3]float64 `json:"points"`
	Error  string       `json:"error,omitempty"`
}

// ToFKResponse builds the API response from a forward-kinematics result.
func ToFKResponse(c ChainSpec, r Result) FKResponse {
	origins := make([][3]float64, len(r.FrameOrigins))
	for i, o := range r.FrameOrigins {
		origins[i] = o.Triple()
	}
	ep := r.EndPosition
	return FKResponse{
		NLinks:      len(c.Links),
		T:           r.T.Slice(),
		EndPosition: [3]float64{ep.X, ep.Y, ep.Z},
		EulerZYX:    r.EulerZYX,
		AxisOrigins: origins,
	}
}

// ToSkeletonResponse builds the API response from a skeleton point list.
func ToSkeletonResponse(pts []dh.Vec3) SkeletonResponse {
	out := make([][3]float64, len(pts))
	for i, p := range pts {
		out[i] = p.Triple()
	}
	return SkeletonResponse{Points: out}
}
