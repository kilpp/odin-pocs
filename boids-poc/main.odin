package boids

// Boids flocking with a core:thread job system.
//
// The per-frame update is double-buffered: every boid reads the CURRENT state
// of all boids (read-only) and writes its new velocity into a separate buffer.
// Because each job only writes its own disjoint index range, the expensive
// O(n^2) neighbour pass parallelises with no locks. Press T to toggle between
// the single-threaded path and the thread.Pool path and watch the compute time.

import "core:fmt"
import "core:math"
import "core:math/rand"
import "core:thread"
import "core:time"
import rl "vendor:raylib"

WINDOW_W :: 1280
WINDOW_H :: 800

MAX_SPEED :: 4.0
MAX_FORCE :: 0.08

NEIGHBOR_R :: 40.0
SEPARATION_R :: 20.0
NEIGHBOR_R2 :: NEIGHBOR_R * NEIGHBOR_R
SEPARATION_R2 :: SEPARATION_R * SEPARATION_R

WORKER_THREADS :: 12 // pool size; the machine has more cores to spare
CHUNK_COUNT :: 48 // > thread count, so faster workers steal extra chunks

Boid :: struct {
	pos: rl.Vector2,
	vel: rl.Vector2,
}

Params :: struct {
	sep: f32,
	ali: f32,
	coh: f32,
}

// One unit of parallel work: compute new velocities for boids[start:end].
// Carries everything the worker needs by value/slice so no globals are touched.
Job :: struct {
	boids:    []Boid,
	new_vel:  []rl.Vector2,
	params:   Params,
	mouse:    rl.Vector2,
	mouse_on: bool,
	start:    int,
	end:      int,
}

main :: proc() {
	rl.InitWindow(WINDOW_W, WINDOW_H, "Odin Boids - threaded job system")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	num_boids := 2000
	boids := make([]Boid, num_boids)
	new_vel := make([]rl.Vector2, num_boids)
	defer delete(boids)
	defer delete(new_vel)
	spawn(boids)

	params := Params{sep = 1.6, ali = 1.0, coh = 0.9}

	jobs := make([]Job, CHUNK_COUNT)
	defer delete(jobs)

	pool: thread.Pool
	thread.pool_init(&pool, context.allocator, WORKER_THREADS)
	defer thread.pool_destroy(&pool)
	thread.pool_start(&pool)

	use_threads := true
	show_help := true
	avg_ms: f64 = 0

	for !rl.WindowShouldClose() {
		mouse_on := rl.IsMouseButtonDown(.LEFT)
		mouse := rl.GetMousePosition()

		// --- input ---
		if rl.IsKeyPressed(.T) do use_threads = !use_threads
		if rl.IsKeyPressed(.H) do show_help = !show_help
		if rl.IsKeyDown(.ONE) do params.sep += 0.02
		if rl.IsKeyDown(.TWO) do params.sep = max(0, params.sep - 0.02)
		if rl.IsKeyDown(.THREE) do params.ali += 0.02
		if rl.IsKeyDown(.FOUR) do params.ali = max(0, params.ali - 0.02)
		if rl.IsKeyDown(.FIVE) do params.coh += 0.02
		if rl.IsKeyDown(.SIX) do params.coh = max(0, params.coh - 0.02)

		// --- update (timed) ---
		start_tick := time.tick_now()

		if use_threads {
			per := (num_boids + CHUNK_COUNT - 1) / CHUNK_COUNT
			for c in 0 ..< CHUNK_COUNT {
				lo := c * per
				if lo >= num_boids do break
				hi := min(lo + per, num_boids)
				jobs[c] = Job {
					boids    = boids,
					new_vel  = new_vel,
					params   = params,
					mouse    = mouse,
					mouse_on = mouse_on,
					start    = lo,
					end      = hi,
				}
				thread.pool_add_task(&pool, context.allocator, boid_task, &jobs[c])
			}
			// Barrier: drain the queue, lending the main thread as a worker too.
			for thread.pool_num_outstanding(&pool) > 0 {
				if t, ok := thread.pool_pop_waiting(&pool); ok {
					thread.pool_do_work(&pool, t)
				} else {
					thread.yield()
				}
			}
			for _, ok := thread.pool_pop_done(&pool); ok; _, ok = thread.pool_pop_done(&pool) {
			}
		} else {
			compute_range(boids, new_vel, params, mouse, mouse_on, 0, num_boids)
		}

		elapsed_ms := time.duration_milliseconds(time.tick_since(start_tick))
		avg_ms = avg_ms * 0.9 + elapsed_ms * 0.1

		// Write phase: apply velocities, advance, wrap at edges.
		for &b, i in boids {
			b.vel = new_vel[i]
			b.pos += b.vel
			if b.pos.x < 0 do b.pos.x += WINDOW_W
			if b.pos.x >= WINDOW_W do b.pos.x -= WINDOW_W
			if b.pos.y < 0 do b.pos.y += WINDOW_H
			if b.pos.y >= WINDOW_H do b.pos.y -= WINDOW_H
		}

		// --- draw ---
		rl.BeginDrawing()
		rl.ClearBackground({12, 14, 20, 255})
		for b in boids {
			draw_boid(b)
		}
		draw_hud(num_boids, use_threads, avg_ms, params, mouse_on, show_help)
		rl.EndDrawing()
	}
}

boid_task :: proc(task: thread.Task) {
	job := cast(^Job)task.data
	compute_range(job.boids, job.new_vel, job.params, job.mouse, job.mouse_on, job.start, job.end)
}

// The hot loop. For each boid in [start,end), scan every other boid and build
// separation / alignment / cohesion steering forces, then write new velocity.
compute_range :: proc(
	boids: []Boid,
	new_vel: []rl.Vector2,
	params: Params,
	mouse: rl.Vector2,
	mouse_on: bool,
	start, end: int,
) {
	for i in start ..< end {
		b := boids[i]
		sep: rl.Vector2
		ali: rl.Vector2
		coh: rl.Vector2
		count: f32

		for j in 0 ..< len(boids) {
			if i == j do continue
			d := boids[j].pos - b.pos
			dist2 := d.x * d.x + d.y * d.y
			if dist2 <= 0 || dist2 > NEIGHBOR_R2 do continue

			ali += boids[j].vel
			coh += boids[j].pos
			count += 1
			if dist2 < SEPARATION_R2 {
				sep -= d / dist2 // push away, stronger when closer
			}
		}

		acc: rl.Vector2
		if count > 0 {
			ali /= count
			coh = coh / count - b.pos
			acc += steer(sep, b.vel) * params.sep
			acc += steer(ali, b.vel) * params.ali
			acc += steer(coh, b.vel) * params.coh
		}
		if mouse_on {
			acc += steer(mouse - b.pos, b.vel) * 1.5
		}

		new_vel[i] = limit(b.vel + acc, MAX_SPEED)
	}
}

// Reynolds steering: desired = normalize(dir)*MAX_SPEED, force = desired - vel.
steer :: proc(dir, vel: rl.Vector2) -> rl.Vector2 {
	if dir == {0, 0} do return {0, 0}
	desired := rl.Vector2Normalize(dir) * MAX_SPEED
	return limit(desired - vel, MAX_FORCE)
}

limit :: proc(v: rl.Vector2, m: f32) -> rl.Vector2 {
	l := rl.Vector2Length(v)
	if l > m && l > 0 do return v / l * m
	return v
}

spawn :: proc(boids: []Boid) {
	for &b in boids {
		b.pos = {rand.float32() * WINDOW_W, rand.float32() * WINDOW_H}
		angle := rand.float32() * 6.2831853
		b.vel = {math.cos(angle) * MAX_SPEED, math.sin(angle) * MAX_SPEED}
	}
}

draw_boid :: proc(b: Boid) {
	// little triangle pointed along velocity
	heading := rl.Vector2Normalize(b.vel)
	if heading == {0, 0} do heading = {1, 0}
	side := rl.Vector2{-heading.y, heading.x}
	tip := b.pos + heading * 7
	l := b.pos - heading * 4 + side * 3
	r := b.pos - heading * 4 - side * 3
	rl.DrawTriangle(tip, l, r, {120, 200, 255, 255})
}

draw_hud :: proc(
	n: int,
	threaded: bool,
	avg_ms: f64,
	p: Params,
	mouse_on: bool,
	show_help: bool,
) {
	rl.DrawFPS(WINDOW_W - 90, 10)
	mode := threaded ? fmt.ctprintf("THREADED x%d", WORKER_THREADS) : "SINGLE-THREAD"
	rl.DrawText(fmt.ctprintf("%s  |  %d boids", mode, n), 10, 10, 20, rl.RAYWHITE)
	rl.DrawText(fmt.ctprintf("update: %.2f ms", avg_ms), 10, 34, 20, {120, 200, 255, 255})
	if show_help {
		rl.DrawText(
			fmt.ctprintf(
				"T threading  |  hold LMB attract  |  H help\n1/2 sep %.2f   3/4 ali %.2f   5/6 coh %.2f",
				p.sep,
				p.ali,
				p.coh,
			),
			10,
			WINDOW_H - 56,
			18,
			{170, 170, 180, 255},
		)
	}
}
