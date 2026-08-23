// Package dh implements 4x4 homogeneous transforms and the standard
// Denavit-Hartenberg (DH) link transform that dh-robot uses for forward
// kinematics of open-chain manipulators.
//
// A homogeneous transform is stored as a row-major [16]float64 so that
// element (row, col) lives at index row*4 + col. Points are column vectors
// and transforms post-multiply: p' = M * p.
package dh

import (
	"fmt"
	"math"
)

// Mat4 is a 4x4 matrix in row-major order.
type Mat4 [16]float64

// Identity returns the 4x4 identity matrix.
func Identity() Mat4 {
	m := Mat4{}
	m[0], m[5], m[10], m[15] = 1, 1, 1, 1
	return m
}

// NewTranslation builds a pure translation matrix that moves a point by (x, y, z).
func NewTranslation(x, y, z float64) Mat4 {
	m := Identity()
	m[3], m[7], m[11] = x, y, z
	return m
}

// At returns the element at (row, col).
func (m Mat4) At(row, col int) float64 { return m[row*4+col] }

// Set assigns the element at (row, col).
func (m *Mat4) Set(row, col int, v float64) { m[row*4+col] = v }

// Mul returns the matrix product a*b using the standard row-dot-column rule.
func (a Mat4) Mul(b Mat4) Mat4 {
	var c Mat4
	for r := 0; r < 4; r++ {
		for col := 0; col < 4; col++ {
			var s float64
			for k := 0; k < 4; k++ {
				s += a[r*4+k] * b[k*4+col]
			}
			c[r*4+col] = s
		}
	}
	return c
}

// MulVec transforms the column vector (x, y, z) with an implicit w = 1 and
// returns the resulting (x', y', z') coordinates.
func (m Mat4) MulVec(x, y, z float64) (nx, ny, nz float64) {
	return m[0]*x + m[1]*y + m[2]*z + m[3],
		m[4]*x + m[5]*y + m[6]*z + m[7],
		m[8]*x + m[9]*y + m[10]*z + m[11]
}

// MulVec4 transforms a full homogeneous column vector (x, y, z, w).
func (m Mat4) MulVec4(x, y, z, w float64) (nx, ny, nz, nw float64) {
	return m[0]*x + m[1]*y + m[2]*z + m[3]*w,
		m[4]*x + m[5]*y + m[6]*z + m[7]*w,
		m[8]*x + m[9]*y + m[10]*z + m[11]*w,
		m[12]*x + m[13]*y + m[14]*z + m[15]*w
}

// Translation returns the (x, y, z) translation column of the matrix.
func (m Mat4) Translation() (x, y, z float64) {
	return m[3], m[7], m[11]
}

// Rotation returns the upper-left 3x3 rotation block as a nested slice.
func (m Mat4) Rotation() [3][3]float64 {
	var r [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			r[i][j] = m[i*4+j]
		}
	}
	return r
}

// ApproxEqual reports whether two matrices match within eps on every element.
func (a Mat4) ApproxEqual(eps float64, b Mat4) bool {
	for i := 0; i < 16; i++ {
		if math.Abs(a[i]-b[i]) > eps {
			return false
		}
	}
	return true
}

// Clone returns a deep copy of the matrix.
func (m Mat4) Clone() Mat4 { return m }

// Slice returns the matrix as a [4][4] slice, convenient for JSON encoding.
func (m Mat4) Slice() [4][4]float64 {
	var s [4][4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			s[i][j] = m[i*4+j]
		}
	}
	return s
}

// String renders the matrix with four rows for debugging.
func (m Mat4) String() string {
	return fmt.Sprintf(
		"[%.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f\n %.4f %.4f %.4f %.4f]",
		m[0], m[1], m[2], m[3],
		m[4], m[5], m[6], m[7],
		m[8], m[9], m[10], m[11],
		m[12], m[13], m[14], m[15],
	)
}
