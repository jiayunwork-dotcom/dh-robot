package dh

import "math"

// Vec3 is a point or free vector in three-dimensional space.
type Vec3 struct {
	X, Y, Z float64
}

// Add returns the component-wise sum of two vectors.
func (v Vec3) Add(o Vec3) Vec3 { return Vec3{v.X + o.X, v.Y + o.Y, v.Z + o.Z} }

// Sub returns the component-wise difference v - o.
func (v Vec3) Sub(o Vec3) Vec3 { return Vec3{v.X - o.X, v.Y - o.Y, v.Z - o.Z} }

// Scale returns the vector scaled by s.
func (v Vec3) Scale(s float64) Vec3 { return Vec3{v.X * s, v.Y * s, v.Z * s} }

// Dot returns the Euclidean dot product of two vectors.
func (v Vec3) Dot(o Vec3) float64 { return v.X*o.X + v.Y*o.Y + v.Z*o.Z }

// Cross returns the cross product v x o.
func (v Vec3) Cross(o Vec3) Vec3 {
	return Vec3{
		v.Y*o.Z - v.Z*o.Y,
		v.Z*o.X - v.X*o.Z,
		v.X*o.Y - v.Y*o.X,
	}
}

// Norm returns the Euclidean length of the vector.
func (v Vec3) Norm() float64 { return math.Sqrt(v.Dot(v)) }

// Dist returns the Euclidean distance between two points.
func (v Vec3) Dist(o Vec3) float64 { return v.Sub(o).Norm() }

// ApproxEqual reports whether two vectors match within eps on every axis.
func (v Vec3) ApproxEqual(o Vec3, eps float64) bool {
	return math.Abs(v.X-o.X) <= eps && math.Abs(v.Y-o.Y) <= eps && math.Abs(v.Z-o.Z) <= eps
}

// Triple returns the vector as a fixed-length [3]float64 array, convenient
// for JSON encoding.
func (v Vec3) Triple() [3]float64 { return [3]float64{v.X, v.Y, v.Z} }
