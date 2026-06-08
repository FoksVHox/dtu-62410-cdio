package navigation

import (
	"image"
	"testing"

	"bot/vision"
)

func TestClamp(t *testing.T) {
	cases := []struct{ v, lo, hi, want float64 }{
		{0.5, 0, 1, 0.5},
		{2.0, 0, 1, 1.0},
		{-0.5, 0, 1, 0},
		{0.0, -1, 1, 0.0},
	}
	for _, c := range cases {
		if got := clamp(c.v, c.lo, c.hi); got != c.want {
			t.Errorf("clamp(%v, %v, %v) = %v, want %v", c.v, c.lo, c.hi, got, c.want)
		}
	}
}

func TestStateString(t *testing.T) {
	if StateSearching.String() != "searching" {
		t.Error("unexpected state string")
	}
	if StateApproaching.String() != "approaching" {
		t.Error("unexpected state string")
	}
	if StateCollecting.String() != "collecting" {
		t.Error("unexpected state string")
	}
}

func TestBallNormX(t *testing.T) {
	b := vision.Ball{Center: image.Pt(320, 240), Radius: 20}
	// Centre of a 640-wide frame should be 0.
	if got := b.NormX(640); got != 0.0 {
		t.Errorf("NormX = %v, want 0", got)
	}
	// Left edge.
	b.Center.X = 0
	if got := b.NormX(640); got != -1.0 {
		t.Errorf("NormX = %v, want -1", got)
	}
	// Right edge.
	b.Center.X = 640
	if got := b.NormX(640); got != 1.0 {
		t.Errorf("NormX = %v, want 1", got)
	}
}

func TestBallEstimatedDistance(t *testing.T) {
	b := vision.Ball{Center: image.Pt(0, 0), Radius: 40}
	dist := b.EstimatedDistance(200)
	want := 5.0
	if dist != want {
		t.Errorf("EstimatedDistance = %v, want %v", dist, want)
	}
}
