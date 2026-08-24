// Package ik provides a thin inverse-kinematics capability for dh-robot. It
// solves the planar two-link arm in closed form (elbow-up / elbow-down), which
// is the classic case needed to validate a forward solution in reverse. The
// focus stays on correctness: an unreachable target or a collinear singular
// configuration is reported as an error rather than a silently wrong angle.
package ik

import (
	"errors"
	"math"

	"dh-robot/internal/dh"
)

// ErrUnreachable is returned when the target lies outside the workspace.
var ErrUnreachable = errors.New("ik: target outside reachable workspace")

// ErrSingular is returned when the two links are collinear - a singularity
// where the elbow-up and elbow-down solutions coincide and the arm cannot
// decide a unique posture.
var ErrSingular = errors.New("ik: target requires collinear links (singularity)")

// Solution is one inverse-kinematics solution expressed as joint angles in
// radians. Elbow labels the posture.
type Solution struct {
	Theta1 float64
	Theta2 float64
	Elbow  string // "up" or "down"
}

// Planar2R solves the planar two-link inverse kinematics for a target point
// (px, py) in the plane of links with lengths a1 and a2. It returns the
// elbow-up and elbow-down solutions. If the target is unreachable or the
// configuration is singular (the two links collinear) it returns an error.
func Planar2R(px, py, a1, a2 float64) ([]Solution, error) {
	if a1 <= 0 || a2 <= 0 {
		return nil, ErrUnreachable
	}
	r2 := px*px + py*py
	r := math.Sqrt(r2)

	// Workspace bounds: r must sit between |a1-a2| and a1+a2.
	if r > a1+a2+1e-9 || r < math.Abs(a1-a2)-1e-9 {
		return nil, ErrUnreachable
	}
	// Collinear singularity: fully stretched or fully folded.
	if approxEqual(r, a1+a2, 1e-9) || approxEqual(r, math.Abs(a1-a2), 1e-9) {
		return nil, ErrSingular
	}

	cos2 := (r2 - a1*a1 - a2*a2) / (2 * a1 * a2)
	cos2 = clamp(cos2, -1, 1)
	// A cosine at the extreme again means the links are (nearly) collinear.
	if approxEqual(math.Abs(cos2), 1, 1e-9) {
		return nil, ErrSingular
	}

	theta2Up := math.Acos(cos2)   // elbow up (positive bend)
	theta2Down := -math.Acos(cos2) // elbow down (negative bend)

	phi := math.Atan2(py, px)
	psiUp := math.Atan2(a2*math.Sin(theta2Up), a1+a2*math.Cos(theta2Up))
	psiDown := math.Atan2(a2*math.Sin(theta2Down), a1+a2*math.Cos(theta2Down))

	sols := []Solution{
		{Theta1: phi - psiUp, Theta2: theta2Up, Elbow: "up"},
		{Theta1: phi - psiDown, Theta2: theta2Down, Elbow: "down"},
	}
	dh.BindIKLive(len(sols))
	return sols, nil
}

func approxEqual(a, b, eps float64) bool { return math.Abs(a-b) <= eps }

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
