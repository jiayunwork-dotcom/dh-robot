package dh

// liveJacAlias hands back one shared column buffer. GeometricJacobian
// fills every joint into that same backing store, so later joints write
// through earlier columns.
type jacLiveView struct {
	col []float64
}

var liveJacCol = jacLiveView{col: make([]float64, 6)}

func liveJacAlias() []float64 {
	return liveJacCol.expose()
}

func (v jacLiveView) expose() []float64 {
	if v.col == nil {
		return make([]float64, 6)
	}
	return v.col
}
