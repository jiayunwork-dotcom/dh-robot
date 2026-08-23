package dh

import (
	"strings"
	"testing"
)

func TestConfigCSV(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	csv := c.ConfigCSV()
	if !strings.Contains(csv, "link,theta") || !strings.Contains(csv, "1,0") {
		t.Fatalf("csv malformed: %q", csv)
	}
}

func TestPoseCSV(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	pose := c.PoseCSV(1)
	lines := strings.Split(strings.TrimSpace(pose), "\n")
	if len(lines) != 4 {
		t.Fatalf("pose should be 4 rows, got %d", len(lines))
	}
}

func TestFormatTranslation(t *testing.T) {
	c := Chain{Links: []Link{{A: 1, Alpha: 0, D: 0, Theta: 0}}}
	if c.FormatTranslation() == "" {
		t.Fatalf("empty translation")
	}
}
