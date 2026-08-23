// Package kin implements forward kinematics for open-chain manipulators
// described by standard Denavit-Hartenberg (DH) parameters. It converts the
// JSON-friendly chain description into the rigid transforms provided by the
// dh package, then extracts the end-effector pose, the per-axis origins and
// the link skeleton.
package kin

import "dh-robot/internal/dh"

// JointType enumerates the supported joint kinds.
type JointType int

const (
	// Revolute joints vary their angle theta; the link offset d is constant.
	Revolute JointType = iota
	// Prismatic joints vary their offset d; the angle theta is constant.
	Prismatic
)

// LinkSpec is the JSON-friendly description of one DH link. Angles are given
// in degrees at this boundary; the kin layer converts them to radians before
// constructing any transform. This pins the angle unit once and for all so a
// caller can never accidentally feed radians where degrees are expected.
type LinkSpec struct {
	A     float64 `json:"a"`     // link length
	Alpha float64 `json:"alpha"` // link twist, degrees
	D     float64 `json:"d"`     // link offset
	Theta float64 `json:"theta"` // joint angle offset, degrees
	Type  string  `json:"type"`  // "revolute" (default) or "prismatic"
}

// ChainSpec fully describes a robot arm: the ordered links, the current joint
// values (degrees for revolute, length for prismatic), an optional fixed tool
// transform, and whether negative link lengths are permitted.
type ChainSpec struct {
	Links          []LinkSpec  `json:"links"`
	Joints         []float64   `json:"joints"`
	TCP            [][]float64 `json:"tcp"` // optional 4x4 row-major tool transform
	AllowNegativeA bool        `json:"allow_negative_a"`
}

// toDH converts a LinkSpec into a dh.Link with angles in radians.
func (l LinkSpec) toDH() dh.Link {
	return dh.Link{
		A:         l.A,
		Alpha:     deg2rad(l.Alpha),
		D:         l.D,
		Theta:     deg2rad(l.Theta),
		Prismatic: l.Type == "prismatic",
	}
}

// jointType normalises the textual joint type.
func (l LinkSpec) jointType() JointType {
	if l.Type == "prismatic" {
		return Prismatic
	}
	return Revolute
}

// dhChain builds the dh.Chain for the spec, applying the current joint
// values. Revolute values are converted from degrees to radians; prismatic
// values are lengths and used verbatim.
func (c ChainSpec) dhChain() dh.Chain {
	links := make([]dh.Link, len(c.Links))
	vars := make([]float64, len(c.Links))
	for i, l := range c.Links {
		links[i] = l.toDH()
		if i < len(c.Joints) {
			if l.jointType() == Prismatic {
				vars[i] = c.Joints[i]
			} else {
				vars[i] = deg2rad(c.Joints[i])
			}
		}
	}
	return dh.Chain{Links: links, Vars: vars}
}
