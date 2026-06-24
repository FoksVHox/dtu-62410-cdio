package main

import "image"

// Ball holds the pixel-space position of a detected table-tennis ball.
type Ball struct {
	Center image.Point
	// InRedZone is true when the ball sits inside a red-zone obstacle rectangle.
	InRedZone bool
	// IsOrange marks the VIP ball that should be delivered first for bonus points.
	IsOrange bool
}

// Phase represents the high-level state of the collection state machine.
type Phase int

const (
	// PhasePickBall: navigate to the next ball and collect it.
	PhasePickBall Phase = iota
	// PhaseDeliverGoal: carry the collected ball(s) to the goal marker and deposit.
	PhaseDeliverGoal
	// PhaseDone: all balls have been delivered — stop everything.
	PhaseDone
)

// CollectionState is the mutable FSM state threaded through the main loop.
type CollectionState struct {
	Phase Phase
	// BallsCollected is the running count of balls successfully delivered to the goal.
	BallsCollected int
	// TotalBalls is the number of balls on the field (11 per the competition rules).
	TotalBalls int
	// OrangeDelivered tracks whether the VIP orange ball bonus has been claimed.
	OrangeDelivered bool
	// CarryingOrange is true while the robot is ferrying the orange ball to the goal.
	CarryingOrange bool
}

// NewCollectionState returns a fresh FSM ready for an 11-ball run.
func NewCollectionState() *CollectionState {
	return &CollectionState{
		Phase:      PhasePickBall,
		TotalBalls: 11,
	}
}
