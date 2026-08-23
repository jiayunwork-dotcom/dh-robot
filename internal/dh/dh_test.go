package dh

import (
	"math"
	"testing"
)

func TestIdentity(t *testing.T) {
	m := Identity()
	for r := 0; r < 4; r++ {
		for c := 0; c < 4; c++ {
			want := 0.0
			if r == c {
				want = 1.0
			}
			if m.At(r, c) != want {
				t.Errorf("Identity(%d,%d) = %g, want %g", r, c, m.At(r, c), want)
			}
		}
	}
}

func TestRotZBasic(t *testing.T) {
	m := RotZ(math.Pi / 2)
	vx, vy, _ := m.MulVec(1, 0, 0)
	if math.Abs(vx) > 1e-9 || math.Abs(vy-1) > 1e-9 {
		t.Errorf("RotZ(90)·(1,0,0) = (%g,%g), want (0,1)", vx, vy)
	}
}

func TestTransXTranslation(t *testing.T) {
	m := TransX(3)
	x, _, _ := m.Translation()
	if math.Abs(x-3) > 1e-9 {
		t.Errorf("TransX(3).Translation = %g, want 3", x)
	}
	vx, _, _ := m.MulVec(0, 0, 0)
	if math.Abs(vx-3) > 1e-9 {
		t.Errorf("TransX(3)·(0,0,0) = %g, want 3", vx)
	}
}

func TestMulComposition(t *testing.T) {
	a := RotZ(0.5)
	b := TransX(2)
	c := a.Mul(b)
	// (RotZ(0.5) * TransX(2)) applied to origin -> rotated (2,0,0)
	x, y, _ := c.MulVec(0, 0, 0)
	if math.Abs(x-2*math.Cos(0.5)) > 1e-9 || math.Abs(y-2*math.Sin(0.5)) > 1e-9 {
		t.Errorf("RotZ*TransX origin = (%g,%g)", x, y)
	}
}

func TestInverse(t *testing.T) {
	m := NewTranslation(1, 2, 3)
	inv := m.Inverse()
	got := m.Mul(inv)
	if !got.ApproxEqual(1e-9, Identity()) {
		t.Errorf("M * M^-1 != I")
	}
}

func TestEulerZYXRoundTrip(t *testing.T) {
	rz := RotZ(0.3)
	ry := RotY(0.7)
	rx := RotX(-0.2)
	m := rz.Mul(ry).Mul(rx)
	e := EulerZYX(m)
	r2 := RotZ(e[0]).Mul(RotY(e[1])).Mul(RotX(e[2]))
	if !m.ApproxEqual(1e-9, r2) {
		t.Errorf("EulerZYX round-trip failed")
	}
}

func TestQuatRoundTrip(t *testing.T) {
	q := QuatFromRotMat(RotZ(0.9))
	r := q.ToRotMat()
	if !r.ApproxEqual(1e-9, RotZ(0.9)) {
		t.Errorf("Quat->RotMat mismatch")
	}
}
