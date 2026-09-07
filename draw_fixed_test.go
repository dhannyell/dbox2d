//go:build !dbox2d_float

package dbox2d

import "testing"

func TestDrawNumberRoundsLabelsWithoutFloat(t *testing.T) {
	for _, tc := range []struct {
		value  string
		places int
		want   string
	}{
		{"1.125", 2, "1.12"}, {"1.375", 2, "1.38"}, {"-1.375", 2, "-1.38"}, {"0", 2, "0.00"}, {"-0.001", 2, "-0.00"}, {"9.999", 2, "10.00"}, {"2147483647.5", 2, "2147483647.50"},
	} {
		if got := drawNumber(QMustParse(tc.value), tc.places); got != tc.want {
			t.Errorf("%s: %s, want %s", tc.value, got, tc.want)
		}
	}
}
