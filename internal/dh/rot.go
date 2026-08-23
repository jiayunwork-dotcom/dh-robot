package dh

import "math"

// RotX returns a rotation of `a` radians about the X axis.
//
//	[1   0      0   ]
//	[0  cos(a) -sin(a)]
//	[0  sin(a)  cos(a)]
func RotX(a float64) Mat4 {
	c, s := math.Cos(a), math.Sin(a)
	m := Identity()
	m[5], m[6] = c, -s
	m[9], m[10] = s, c
	return m
}

// RotY returns a rotation of `a` radians about the Y axis.
//
//	[ cos(a)  0  sin(a)]
//	[   0     1    0   ]
//	[-sin(a)  0  cos(a)]
func RotY(a float64) Mat4 {
	c, s := math.Cos(a), math.Sin(a)
	m := Identity()
	m[0], m[2] = c, s
	m[8], m[10] = -s, c
	return m
}

// RotZ returns a rotation of `a` radians about the Z axis.
//
//	[cos(a) -sin(a)  0]
//	[sin(a)  cos(a)  0]
//	[  0       0     1]
func RotZ(a float64) Mat4 {
	c, s := math.Cos(a), math.Sin(a)
	m := Identity()
	m[0], m[1] = c, -s
	m[4], m[5] = s, c
	return m
}
