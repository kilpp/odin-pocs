package boids

// Minimal raylib POC: two balls bouncing off the walls and each other.

import rl "vendor:raylib"

RADIUS :: 20

Ball :: struct {
	pos:   rl.Vector2,
	vel:   rl.Vector2,
	color: rl.Color,
}

main :: proc() {
	rl.InitWindow(800, 600, "Odin POC")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	balls := [2]Ball {
		{pos = {200, 300}, vel = {4, 2}, color = rl.RAYWHITE},
		{pos = {600, 300}, vel = {-3, -2}, color = rl.SKYBLUE},
	}

	for !rl.WindowShouldClose() {
		// Move + bounce off walls.
		for &b in balls {
			b.pos += b.vel
			if b.pos.x < RADIUS || b.pos.x > 800 - RADIUS do b.vel.x = -b.vel.x
			if b.pos.y < RADIUS || b.pos.y > 600 - RADIUS do b.vel.y = -b.vel.y
		}

		// Ball-vs-ball collision (equal mass: swap velocity along the normal).
		delta := balls[1].pos - balls[0].pos
		dist := rl.Vector2Length(delta)
		if dist > 0 && dist < RADIUS * 2 {
			n := delta / dist
			// Push them apart so they don't stick together.
			overlap := RADIUS * 2 - dist
			balls[0].pos -= n * (overlap / 2)
			balls[1].pos += n * (overlap / 2)
			// Exchange the velocity components along the collision normal.
			v0 := rl.Vector2DotProduct(balls[0].vel, n)
			v1 := rl.Vector2DotProduct(balls[1].vel, n)
			balls[0].vel += n * (v1 - v0)
			balls[1].vel += n * (v0 - v1)
		}

		rl.BeginDrawing()
		rl.ClearBackground({12, 14, 20, 255})
		for b in balls {
			rl.DrawCircleV(b.pos, RADIUS, b.color)
		}
		rl.EndDrawing()
	}
}
