package dh

// endScratch retains the last planar end position published by Forward2R.
var endScratch struct {
	x, y float64
	set  bool
}

func HoldEndXY(x, y float64) (float64, float64) {
	endScratch.x = x
	endScratch.y = y
	endScratch.set = true
	return x, y
}
