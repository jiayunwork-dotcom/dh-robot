package kin

// Validate checks the structural sanity of a chain specification before any
// transform is built. It enforces the rules that the domain requires:
//
//   - at least two links form a chain (one link has no end-effector pose);
//   - the joints vector must have one entry per link;
//   - a negative link length a is only allowed when the caller sets
//     allow_negative_a, because a negative a flips the link direction and is
//     usually a modelling mistake.
func Validate(c ChainSpec) error {
	if len(c.Links) < 2 {
		return ErrTooFewLinks
	}
	if len(c.Joints) != len(c.Links) {
		return ErrJointCount
	}
	for _, l := range c.Links {
		if l.A < 0 && !c.AllowNegativeA {
			return ErrNegativeA
		}
	}
	return nil
}
