package kin

import "math"

// degPerRad is the conversion factor from degrees to radians.
const degPerRad = math.Pi / 180.0

// deg2rad converts an angle in degrees to radians.
func deg2rad(d float64) float64 { return d * degPerRad }

// rad2deg converts an angle in radians to degrees.
func rad2deg(r float64) float64 { return r / degPerRad }
