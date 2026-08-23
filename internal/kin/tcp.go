package kin

import (
	"dh-robot/internal/dh"
	"fmt"
)

// tcpFromSlice converts a 4x4 row-major slice into a dh.Mat4. An empty or
// malformed slice falls back to the identity transform, which is the default
// tool frame (the end-effector coincides with the last link frame).
func tcpFromSlice(tcp [][]float64) dh.Mat4 {
	if len(tcp) == 0 {
		return dh.Identity()
	}
	if len(tcp) != 4 {
		return dh.Identity()
	}
	var m dh.Mat4
	for r := 0; r < 4; r++ {
		if len(tcp[r]) != 4 {
			return dh.Identity()
		}
		for col := 0; col < 4; col++ {
			m[r*4+col] = tcp[r][col]
		}
	}
	return m
}

// ApplyTCP multiplies a base transform by the tool transform.
func ApplyTCP(base, tcp dh.Mat4) dh.Mat4 {
	return base.Mul(tcp)
}

// DescribeTCP returns a short human-readable description of a tool transform
// for use in summaries and logs.
func DescribeTCP(tcp dh.Mat4) string {
	x, y, z := tcp.Translation()
	return fmt.Sprintf("tool=(%.4f,%.4f,%.4f)", x, y, z)
}
