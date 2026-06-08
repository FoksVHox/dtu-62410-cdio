package vision

import (
	"image"
	"math"
	"testing"
)

func TestBallNormX(t *testing.T) {
	b := Ball{Center: image.Pt(320, 240), Radius: 15}
	if got := b.NormX(640); got != 0 {
		t.Errorf("expected 0, got %v", got)
	}
	b.Center.X = 0
	if got := b.NormX(640); got != -1 {
		t.Errorf("expected -1, got %v", got)
	}
}

func TestBallNormY(t *testing.T) {
	b := Ball{Center: image.Pt(320, 240), Radius: 15}
	if got := b.NormY(480); got != 0 {
		t.Errorf("expected 0, got %v", got)
	}
}

func TestBallNormEdgeCases(t *testing.T) {
	b := Ball{}
	if b.NormX(0) != 0 {
		t.Error("zero width should return 0")
	}
	if b.NormY(0) != 0 {
		t.Error("zero height should return 0")
	}
}

func TestEstimatedDistance(t *testing.T) {
	b := Ball{Radius: 40}
	// With K=200, distance should be 5.0.
	got := b.EstimatedDistance(200)
	if got != 5.0 {
		t.Errorf("expected 5.0, got %v", got)
	}

	// Zero radius should return MaxFloat64.
	b.Radius = 0
	if got := b.EstimatedDistance(200); got != math.MaxFloat64 {
		t.Errorf("expected MaxFloat64, got %v", got)
	}
}

func TestSortBalls(t *testing.T) {
	balls := []Ball{
		{Radius: 10},
		{Radius: 30},
		{Radius: 20},
	}
	sortBalls(balls)
	if balls[0].Radius != 30 || balls[1].Radius != 20 || balls[2].Radius != 10 {
		t.Errorf("sort order wrong: %v", balls)
	}
}
