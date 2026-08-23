package dh

import "math"

// EulerZYX extracts the ZYX (a.k.a. yaw-pitch-roll) Euler angles from the
// rotation part of a homogeneous matrix. The returned triple is in radians
// and ordered as [yaw (rotation about Z), pitch (about Y), roll (about X)].
//
// Given the rotation matrix R:
//
//	yaw   = atan2(R21, R11)
//	pitch = atan2(-R31, sqrt(R32^2 + R33^2))
//	roll  = atan2(R32, R33)
//
// dh-robot pins this single convention so that callers always interpret
// orientation the same way.
func EulerZYX(m Mat4) [3]float64 {
	r := m.Rotation()
	yaw := math.Atan2(r[1][0], r[0][0])
	pitch := math.Atan2(-r[2][0], math.Sqrt(r[2][1]*r[2][1]+r[2][2]*r[2][2]))
	roll := math.Atan2(r[2][1], r[2][2])
	return [3]float64{yaw, pitch, roll}
}
