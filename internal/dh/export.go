package dh

import (
	"fmt"
	"strings"
)

// ConfigCSV renders the current chain links as a comma-separated table of the
// four DH parameters, one row per link. It is the textual counterpart of the
// numeric model and is handy for logging or diffing configurations.
func (c Chain) ConfigCSV() string {
	var b strings.Builder
	b.WriteString("link,theta,alpha,a,d,prismatic\n")
	for i, l := range c.Links {
		fmt.Fprintf(&b, "%d,%.6g,%.6g,%.6g,%.6g,%t\n", i, l.Theta, l.Alpha, l.A, l.D, l.Prismatic)
	}
	return b.String()
}

// PoseCSV renders the 4x4 homogeneous matrix of the nth frame (0 = base, n =
// end) as 4 lines of space-separated floats. Callers use it to pipe poses into
// downstream tools that expect row-major matrices.
func (c Chain) PoseCSV(frame int) string {
	cum := c.Cumulative()
	if frame < 0 || frame > len(c.Links) {
		frame = len(c.Links)
	}
	m := cum[frame]
	var b strings.Builder
	for r := 0; r < 4; r++ {
		row := make([]string, 4)
		for col := 0; col < 4; col++ {
			row[col] = fmt.Sprintf("%.6g", m[r*4+col])
		}
		b.WriteString(strings.Join(row, " ") + "\n")
	}
	return b.String()
}

// FormatTranslation returns the end-effector position as "(x, y, z)" with three
// decimals, for compact human-readable reporting.
func (c Chain) FormatTranslation() string {
	x, y, z := c.Cumulative()[len(c.Links)].Translation()
	return fmt.Sprintf("(%.3f, %.3f, %.3f)", x, y, z)
}
