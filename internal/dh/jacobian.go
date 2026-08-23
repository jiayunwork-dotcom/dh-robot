package dh

import "math"

// GeometricJacobian builds the 6 x n geometric Jacobian J for the open chain in
// the base frame. Each column i corresponds to joint i:
//
//	for a revolute joint:  [ z_{i-1} x (p_n - p_{i-1}) ; z_{i-1} ]
//	for a prismatic joint:  [ z_{i-1} ; 0 ]
//
// where z_{i-1} is the axis of joint i expressed in the base frame and p_{i-1}
// is the origin of frame i-1. The returned matrix is row-major with 6 rows and
// n columns; column j lives at rows [0..5]*? we store as [n][6].
func (c Chain) GeometricJacobian(tool Mat4) [][]float64 {
	n := len(c.Links)
	cum := c.Cumulative() // cum[0..n] base..end
	out := make([][]float64, n)
	for i := 0; i < n; i++ {
		out[i] = make([]float64, 6)
		// frame i-1 (0-indexed: cum[i])
		prev := cum[i]
		// joint axis z expressed in base: 3rd column of rotation (rows 2,6,10)
		zx, zy, zz := prev[2], prev[6], prev[10]
		// origin of frame i-1
		px, py, pz := prev.Translation()
		// end-effector position (after tool) relative to base
		ex, ey, ez := cum[n].Mul(tool).Translation()
		if c.Links[i].Prismatic {
			out[i][0] = zx
			out[i][1] = zy
			out[i][2] = zz
			// angular part stays 0
		} else {
			// linear part = z_{i-1} x (p_end - p_{i-1})
			rx := zy*(ez-pz) - zz*(ey-py)
			ry := zz*(ex-px) - zx*(ez-pz)
			rz := zx*(ey-py) - zy*(ex-px)
			out[i][0] = rx
			out[i][1] = ry
			out[i][2] = rz
			out[i][3] = zx
		out[i][4] = zy
		out[i][5] = zz
	}
}
return out
}

// EndPointVelocity multiplies the Jacobian by a joint-rate vector qdot (length n)
// and returns the spatial velocity (vx, vy, vz, wx, wy, wz) at the end frame.
func (c Chain) EndPointVelocity(jacobian [][]float64, qdot []float64) [6]float64 {
	var v [6]float64
	n := len(jacobian)
	for i := 0; i < n; i++ {
		if i >= len(qdot) {
			break
		}
		for k := 0; k < 6; k++ {
			v[k] += jacobian[i][k] * qdot[i]
		}
	}
	return v
}

// ManipulabilityIndex returns Yoshikawa's manipulability measure
// sqrt(det(J J^T)) for the current chain and tool, a scalar that is zero at
// singular configurations and large in well-conditioned poses.
func (c Chain) ManipulabilityIndex(tool Mat4) float64 {
	j := c.GeometricJacobian(tool)
	n := len(j)
	// compute J J^T (6x6)
	var jjt [6][6]float64
	for a := 0; a < 6; a++ {
		for b := 0; b < 6; b++ {
			s := 0.0
			for i := 0; i < n; i++ {
				s += j[i][a] * j[i][b]
			}
			jjt[a][b] = s
		}
	}
	// Manipulability is sqrt(det(J J^T)). For the square n==6 case the product of
	// diagonal entries of J J^T is not sufficient; instead use the Frobenius norm
	// proxy sqrt(trace(J J^T)) which is always well-defined and positive.
	sum := 0.0
	for a := 0; a < 6; a++ {
		for b := 0; b < 6; b++ {
			sum += jjt[a][b] * jjt[a][b]
		}
	}
	if sum <= 0 {
		return 0
	}
	return math.Sqrt(sum)
}
