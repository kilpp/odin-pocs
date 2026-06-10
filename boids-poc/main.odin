package boids

// Boids flocking simulation (Craig Reynolds, 1986).
// Each boid steers using three local rules:
//   - separation: steer away from crowding neighbours
//   - alignment:  steer toward the average heading of neighbours
//   - cohesion:   steer toward the average position of neighbours
//
// Build & run:  odin run . -o:speed

import "core:fmt"
import "core:math"
import "core:math/rand"
import rl "vendor:raylib"

WINDOW_W :: 1280
WINDOW_H :: 800
BOID_COUNT :: 300

Boid :: struct {
	pos: rl.Vector2,
	vel: rl.Vector2,
}

// Tunable rule weights / radii. Exposed so we can tweak them live with keys.
Config :: struct {
	max_speed:        f32,
	max_force:        f32,
	perception:       f32, // neighbour radius for alignment + cohesion
	separation_range: f32, // tighter radius for separation
	sep_weight:       f32,
	ali_weight:       f32,
	coh_weight:       f32,
}

default_config :: proc() -> Config {
	return Config {
		max_speed        = 4.0,
		max_force        = 0.08,
		perception       = 60.0,
		separation_range = 28.0,
		sep_weight       = 1.6,
		ali_weight       = 1.0,
		coh_weight       = 1.0,
	}
}

// Clamp a vector's magnitude to `max`, preserving direction.
limit :: proc(v: rl.Vector2, max: f32) -> rl.Vector2 {
	len := rl.Vector2Length(v)
	if len > max && len > 0 {
		return v * (max / len)
	}
	return v
}

// Return a steering force that drives current velocity toward `desired`.
steer_toward :: proc(desired, vel: rl.Vector2, cfg: Config) -> rl.Vector2 {
	if rl.Vector2Length(desired) == 0 {
		return {0, 0}
	}
	d := rl.Vector2Normalize(desired) * cfg.max_speed
	return limit(d - vel, cfg.max_force)
}

update_boid :: proc(boids: []Boid, i: int, cfg: Config) -> rl.Vector2 {
	b := boids[i]

	sep_sum: rl.Vector2
	ali_sum: rl.Vector2
	coh_sum: rl.Vector2
	ali_count: int
	coh_count: int
	sep_count: int

	for other, j in boids {
		if i == j {
			continue
		}
		offset := b.pos - other.pos
		dist := rl.Vector2Length(offset)
		if dist == 0 {
			continue
		}
		if dist < cfg.separation_range {
			// Weight by inverse distance so very close boids push harder.
			sep_sum += rl.Vector2Normalize(offset) / dist
			sep_count += 1
		}
		if dist < cfg.perception {
			ali_sum += other.vel
			coh_sum += other.pos
			ali_count += 1
			coh_count += 1
		}
	}

	acc: rl.Vector2

	if sep_count > 0 {
		sep_sum /= f32(sep_count)
		acc += steer_toward(sep_sum, b.vel, cfg) * cfg.sep_weight
	}
	if ali_count > 0 {
		ali_sum /= f32(ali_count)
		acc += steer_toward(ali_sum, b.vel, cfg) * cfg.ali_weight
	}
	if coh_count > 0 {
		coh_sum /= f32(coh_count)
		toward := coh_sum - b.pos
		acc += steer_toward(toward, b.vel, cfg) * cfg.coh_weight
	}

	return acc
}

// Wrap a boid around screen edges so the flock never leaves the view.
wrap :: proc(p: ^rl.Vector2) {
	if p.x < 0 do p.x += WINDOW_W
	if p.x > WINDOW_W do p.x -= WINDOW_W
	if p.y < 0 do p.y += WINDOW_H
	if p.y > WINDOW_H do p.y -= WINDOW_H
}

draw_boid :: proc(b: Boid) {
	// Draw a small triangle pointing along the velocity.
	heading := rl.Vector2Normalize(b.vel)
	if rl.Vector2Length(b.vel) == 0 {
		heading = {1, 0}
	}
	perp := rl.Vector2{-heading.y, heading.x}

	size: f32 = 7.0
	tip := b.pos + heading * size
	left := b.pos - heading * (size * 0.5) + perp * (size * 0.4)
	right := b.pos - heading * (size * 0.5) - perp * (size * 0.4)

	// Colour by heading angle for a bit of visual life.
	angle := math.atan2(heading.y, heading.x)
	hue := (angle / math.PI + 1) * 0.5 * 360
	color := rl.ColorFromHSV(hue, 0.7, 1.0)

	rl.DrawTriangle(tip, left, right, color)
}

main :: proc() {
	rl.InitWindow(WINDOW_W, WINDOW_H, "Odin Boids")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	cfg := default_config()

	boids := make([]Boid, BOID_COUNT)
	defer delete(boids)

	for &b in boids {
		b.pos = {rand.float32() * WINDOW_W, rand.float32() * WINDOW_H}
		angle := rand.float32() * 2 * math.PI
		b.vel = {math.cos(angle), math.sin(angle)} * cfg.max_speed
	}

	// Scratch buffer for accelerations so all boids update from the same frame.
	acc := make([]rl.Vector2, BOID_COUNT)
	defer delete(acc)

	show_help := true

	for !rl.WindowShouldClose() {
		// --- live tuning ---
		if rl.IsKeyPressed(.H) do show_help = !show_help
		if rl.IsKeyDown(.ONE) do cfg.sep_weight = clamp(cfg.sep_weight + 0.02, 0, 5)
		if rl.IsKeyDown(.TWO) do cfg.sep_weight = clamp(cfg.sep_weight - 0.02, 0, 5)
		if rl.IsKeyDown(.THREE) do cfg.ali_weight = clamp(cfg.ali_weight + 0.02, 0, 5)
		if rl.IsKeyDown(.FOUR) do cfg.ali_weight = clamp(cfg.ali_weight - 0.02, 0, 5)
		if rl.IsKeyDown(.FIVE) do cfg.coh_weight = clamp(cfg.coh_weight + 0.02, 0, 5)
		if rl.IsKeyDown(.SIX) do cfg.coh_weight = clamp(cfg.coh_weight - 0.02, 0, 5)

		// --- simulation step ---
		for _, i in boids {
			acc[i] = update_boid(boids[:], i, cfg)
		}

		// Mouse acts as a gentle attractor when left button is held.
		mouse_active := rl.IsMouseButtonDown(.LEFT)
		mouse := rl.GetMousePosition()

		for &b, i in boids {
			b.vel = limit(b.vel + acc[i], cfg.max_speed)
			if mouse_active {
				pull := steer_toward(mouse - b.pos, b.vel, cfg) * 1.2
				b.vel = limit(b.vel + pull, cfg.max_speed)
			}
			b.pos += b.vel
			wrap(&b.pos)
		}

		// --- render ---
		rl.BeginDrawing()
		rl.ClearBackground({12, 14, 20, 255})

		for b in boids {
			draw_boid(b)
		}

		rl.DrawFPS(10, 10)
		if show_help {
			y: i32 = 36
			line :: proc(y: ^i32, text: cstring) {
				rl.DrawText(text, 10, y^, 18, rl.LIGHTGRAY)
				y^ += 22
			}
			line(&y, "Hold LEFT MOUSE: attract flock")
			line(&y, "1/2: separation +/-   3/4: alignment +/-   5/6: cohesion +/-")
			line(&y, "H: toggle help")
			rl.DrawText(
				rl.TextFormat("sep %.2f  ali %.2f  coh %.2f", cfg.sep_weight, cfg.ali_weight, cfg.coh_weight),
				10, y, 18, rl.YELLOW,
			)
		}

		rl.EndDrawing()
	}

	fmt.println("bye")
}
