package main

import "image"

// Ball holds the pixel-space position of a detected table-tennis ball.
type Ball struct {
	Center image.Point
	// InRedZone is true when the ball sits inside a red-zone obstacle rectangle.
	InRedZone bool
}
