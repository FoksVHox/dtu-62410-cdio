package vision

import "time"

type RobotPose struct {
	Detected bool    `json:"detected"`
	X        float64 `json:"x"`
	Y        float64 `json:"y"`
	Heading  float64 `json:"heading"`
	BodyX    int     `json:"body_x"`
	BodyY    int     `json:"body_y"`
	FrontX   int     `json:"front_x"`
	FrontY   int     `json:"front_y"`
}

type BallState struct {
	X      int     `json:"x"`
	Y      int     `json:"y"`
	NormX  float64 `json:"norm_x"`
	NormY  float64 `json:"norm_y"`
	Radius float64 `json:"radius"`
}

type WorldState struct {
	Timestamp   time.Time   `json:"timestamp"`
	FrameWidth  int         `json:"frame_width"`
	FrameHeight int         `json:"frame_height"`
	Robot       RobotPose   `json:"robot"`
	Balls       []BallState `json:"balls"`
}
