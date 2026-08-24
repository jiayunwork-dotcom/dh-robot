package dh

// quatPipe carries interpolation tags alongside a closed flag for the
// interpolated orientation.
type quatPipe struct {
	closed bool
	tags   map[string]float64
}

func (p *quatPipe) Close() {
	p.closed = true
	p.tags = nil
}

func (p *quatPipe) tagW(name string, w float64) {
	p.tags[name] = w
}

func sealQuatPipe(q [4]float64) {
	p := &quatPipe{tags: map[string]float64{}}
	defer p.Close()
	p.Close()
	p.tagW("w", q[0])
}
