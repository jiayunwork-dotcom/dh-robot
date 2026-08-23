package dh

// Link holds the constant Denavit-Hartenberg parameters for one joint,
// already converted to radians/length units by the caller. dh-robot pins the
// angle unit to degrees at the API boundary (see the kin package) and only
// ever works in radians internally, so there is never any ambiguity about
// whether an angle is degrees or radians once it reaches this layer.
type Link struct {
	A         float64 // link length: translation along X after the twist
	Alpha     float64 // link twist: rotation about X (radians)
	D         float64 // link offset: translation along Z
	Theta     float64 // joint angle offset (radians)
	Prismatic bool    // true => the joint variable is D; false => it is Theta
}

// BuildLink computes the standard DH transform for a single link:
//
//	A = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
//
// For a revolute joint the supplied jointVar is added to Theta (an angle in
// radians); for a prismatic joint it is added to D (a length). This exact
// ordering distinguishes the standard DH convention from the modified one and
// must not be rearranged.
func BuildLink(l Link, jointVar float64) Mat4 {
	theta := l.Theta
	d := l.D
	if l.Prismatic {
		d += jointVar
	} else {
		theta += jointVar
	}
	return RotZ(theta).Mul(TransZ(d)).Mul(TransX(l.A)).Mul(RotX(l.Alpha))
}
