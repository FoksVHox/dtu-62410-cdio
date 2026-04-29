package navigation

type Point struct {
	X float64
	Y float64
}

type CourseState struct {
	Robot RobotState
	Balls []Point
	Goal  Point
}

type RobotState struct {
	Position Point
	Heading  float64 // angle in degrees, 0 = right, 90 = up, etc
}
