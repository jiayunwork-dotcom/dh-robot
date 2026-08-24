package kin

import (
	"math"
	"testing"

	"dh-robot/internal/dh"
)

func TestValidateConfigLength(t *testing.T) {
	c := ChainSpec{Links: []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	errs := c.ValidateConfig([]float64{1, 2}, nil)
	if len(errs) != 1 {
		t.Fatalf("expected length error, got %v", errs)
	}
}

func TestValidateConfigNaN(t *testing.T) {
	c := ChainSpec{Links: []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	errs := c.ValidateConfig([]float64{math.NaN()}, nil)
	if len(errs) != 1 {
		t.Fatalf("expected NaN error")
	}
}

func TestValidateConfigLimit(t *testing.T) {
	c := ChainSpec{Links: []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	limits := []dh.JointLimit{{Min: -1, Max: 1}}
	errs := c.ValidateConfig([]float64{5}, limits)
	if len(errs) != 1 {
		t.Fatalf("expected limit error, got %v", errs)
	}
}

func TestNearestValidClamps(t *testing.T) {
	c := ChainSpec{Links: []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	limits := []dh.JointLimit{{Min: -1, Max: 1}}
	out, changed := c.NearestValid([]float64{5}, limits)
	if !changed || out[0] != 1 {
		t.Fatalf("expected clamp to 1, got %v changed=%v", out, changed)
	}
}

func TestSummaryLine(t *testing.T) {
	c := ChainSpec{Links: []LinkSpec{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	s := c.SummaryLine([]float64{0})
	if len(s) == 0 {
		t.Fatalf("summary empty")
	}
}
