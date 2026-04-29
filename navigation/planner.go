package navigation

import "math"

func Distance(a, b Point) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(dx*dx + dy*dy)
}

// PickNextBall returns the best ball to go for next.
// Prefers balls that are close AND roughly between the robot and goal.
func PickNextBall(state CourseState) (Point, bool) {
	if len(state.Balls) == 0 {
		return Point{}, false
	}

	best := state.Balls[0]
	bestScore := math.MaxFloat64

	for _, ball := range state.Balls {
		dist := Distance(state.Robot.Position, ball)
		// Bonus: penalize balls that are far from the goal line
		distToGoal := Distance(ball, state.Goal)
		score := dist + distToGoal*0.3 // weight is tunable,higher means it prioritizes balls near the goal more aggressively
		if score < bestScore {
			bestScore = score
			best = ball
		}
	}
	return best, true
}
