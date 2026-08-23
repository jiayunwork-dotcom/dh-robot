package kin

import "errors"

// Validation and computation errors returned by the kinematics layer. They
// are returned (never panicked) so that callers - including the HTTP API -
// can surface a precise message to the user.
var (
	// ErrTooFewLinks is returned when fewer than two links are supplied. A
	// single link cannot form a chain with a meaningful end-effector.
	ErrTooFewLinks = errors.New("dh-robot: at least two links are required")

	// ErrJointCount is returned when the joint vector length does not match
	// the number of links.
	ErrJointCount = errors.New("dh-robot: joints length must equal number of links")

	// ErrNegativeA is returned when a link length a is negative without the
	// explicit allow_negative_a flag.
	ErrNegativeA = errors.New("dh-robot: negative link length a requires allow_negative_a=true")

	// ErrBadFrame is returned when a frame index is outside the chain.
	ErrBadFrame = errors.New("dh-robot: frame index out of range")
)
