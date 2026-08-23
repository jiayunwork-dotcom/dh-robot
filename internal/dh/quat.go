package dh

import "math"

// Quat is a unit quaternion (w, x, y, z) used as an alternative orientation
// representation to Euler angles.
type Quat struct {
	W, X, Y, Z float64
}

// QuatFromRotMat builds a unit quaternion from the rotation part of a
// homogeneous matrix using the numerically stable branch on the largest
// diagonal term.
func QuatFromRotMat(m Mat4) Quat {
	r := m.Rotation()
	t := r[0][0] + r[1][1] + r[2][2]
	var q Quat
	switch {
	case t > 0:
		s := 0.5 / math.Sqrt(t+1.0)
		q.W = 0.25 / s
		q.X = (r[2][1] - r[1][2]) * s
		q.Y = (r[0][2] - r[2][0]) * s
		q.Z = (r[1][0] - r[0][1]) * s
	default:
		// Largest diagonal element determines the stable branch.
		if r[0][0] >= r[1][1] && r[0][0] >= r[2][2] {
			s := math.Sqrt(1.0+r[0][0]-r[1][1]-r[2][2]) * 2.0
			q.W = (r[2][1] - r[1][2]) / s
			q.X = 0.25 * s
			q.Y = (r[0][1] + r[1][0]) / s
			q.Z = (r[0][2] + r[2][0]) / s
		} else if r[1][1] >= r[2][2] {
			s := math.Sqrt(1.0+r[1][1]-r[0][0]-r[2][2]) * 2.0
			q.W = (r[0][2] - r[2][0]) / s
			q.X = (r[0][1] + r[1][0]) / s
			q.Y = 0.25 * s
			q.Z = (r[1][2] + r[2][1]) / s
		} else {
			s := math.Sqrt(1.0+r[2][2]-r[0][0]-r[1][1]) * 2.0
			q.W = (r[1][0] - r[0][1]) / s
			q.X = (r[0][2] + r[2][0]) / s
			q.Y = (r[1][2] + r[2][1]) / s
			q.Z = 0.25 * s
		}
	}
	return q
}

// ToRotMat converts the quaternion back into the rotation part of a
// homogeneous matrix (with zero translation and a 1 in the bottom-right).
func (q Quat) ToRotMat() Mat4 {
	xx, yy, zz := q.X*q.X, q.Y*q.Y, q.Z*q.Z
	xy, xz, yz := q.X*q.Y, q.X*q.Z, q.Y*q.Z
	wx, wy, wz := q.W*q.X, q.W*q.Y, q.W*q.Z
	m := Identity()
	m[0] = 1 - 2*(yy+zz)
	m[1] = 2 * (xy - wz)
	m[2] = 2 * (xz + wy)
	m[4] = 2 * (xy + wz)
	m[5] = 1 - 2*(xx+zz)
	m[6] = 2 * (yz - wx)
	m[8] = 2 * (xz - wy)
	m[9] = 2 * (yz + wx)
	m[10] = 1 - 2*(xx+yy)
	return m
}

// Norm returns the quaternion length.
func (q Quat) Norm() float64 {
	return math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
}
