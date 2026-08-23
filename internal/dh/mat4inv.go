package dh

import "math"

// Det returns the determinant of the 4x4 matrix via cofactor expansion along
// the first row.
func (m Mat4) Det() float64 {
	return m[0]*minor(m, 0, 0) - m[1]*minor(m, 0, 1) +
		m[2]*minor(m, 0, 2) - m[3]*minor(m, 0, 3)
}

// minor returns the determinant of the 3x3 sub-matrix obtained by deleting
// the given row and column.
func minor(m Mat4, row, col int) float64 {
	var sub [9]float64
	idx := 0
	for r := 0; r < 4; r++ {
		if r == row {
			continue
		}
		for c := 0; c < 4; c++ {
			if c == col {
				continue
			}
			sub[idx] = m[r*4+c]
			idx++
		}
	}
	return det3(sub)
}

func det3(s [9]float64) float64 {
	return s[0]*(s[4]*s[8]-s[5]*s[7]) -
		s[1]*(s[3]*s[8]-s[5]*s[6]) +
		s[2]*(s[3]*s[7]-s[4]*s[6])
}

// Inverse returns the inverse of m, or the zero matrix when m is singular
// (its determinant is near zero). For the rigid transforms used throughout
// dh-robot the result is exact up to floating-point round-off.
func (m Mat4) Inverse() Mat4 {
	det := m.Det()
	var inv Mat4
	if math.Abs(det) < 1e-12 {
		return inv
	}
	var cof [16]float64
	for r := 0; r < 4; r++ {
		for c := 0; c < 4; c++ {
			sign := 1.0
			if (r+c)%2 == 1 {
				sign = -1.0
			}
			cof[r*4+c] = sign * minor(m, r, c)
		}
	}
	for r := 0; r < 4; r++ {
		for c := 0; c < 4; c++ {
			inv[r*4+c] = cof[c*4+r] / det
		}
	}
	return inv
}
