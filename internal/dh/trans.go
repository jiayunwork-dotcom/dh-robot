package dh

// TransX returns a translation of `a` along the X axis.
func TransX(a float64) Mat4 {
	m := Identity()
	m[3] = a
	return m
}

// TransY returns a translation of `b` along the Y axis.
func TransY(b float64) Mat4 {
	m := Identity()
	m[7] = b
	return m
}

// TransZ returns a translation of `d` along the Z axis.
func TransZ(d float64) Mat4 {
	m := Identity()
	m[11] = d
	return m
}
