# boids-poc

A boids flocking simulation in [Odin](https://odin-lang.org), rendered with raylib
(bundled in Odin's `vendor:raylib`). Each boid steers using Craig Reynolds' three
classic rules: **separation**, **alignment**, and **cohesion**.

The point of this POC is the **`core:thread` job system**: the O(n²) neighbour
pass is split across a `thread.Pool` so it scales across cores. Press `T` to flip
between the single-threaded and pooled paths and watch the per-frame update time.

## Run

```sh
cd boids-poc
odin run . -o:speed
```

## Controls

| Input            | Action                                |
| ---------------- | ------------------------------------- |
| `T`              | Toggle single-thread vs threaded pool |
| Hold left mouse  | Attract the flock to the cursor       |
| `1` / `2`        | Separation weight + / -               |
| `3` / `4`        | Alignment weight + / -                |
| `5` / `6`        | Cohesion weight + / -                 |
| `H`              | Toggle the help overlay               |

The HUD shows the current mode, boid count, and a smoothed `update: X.XX ms`
that measures only the steering compute (not draw), so the threaded speedup is
directly visible.

## How the parallelism works

The update is **double-buffered**, which is what makes it safe to parallelise
without locks:

1. **Read phase (the expensive part).** Every boid reads the *current* state of
   all boids and writes its new velocity into a separate `new_vel` buffer. Reads
   are shared and read-only; each job writes only its own disjoint index range,
   so there are no data races and no mutexes in the hot loop.
2. **Write phase (cheap).** The main thread applies `new_vel` → position and
   wraps boids at the screen edges.

The job system slices the flock into `CHUNK_COUNT` ranges and feeds one
`thread.Task` per chunk to a `thread.Pool` of `WORKER_THREADS`. Using more
chunks than threads lets faster workers pick up extra work (cheap load
balancing). Each frame the main thread acts as a barrier: it drains the queue —
lending itself as an extra worker via `pool_do_work` — until
`pool_num_outstanding` hits zero, then applies the results.

## What this exercises in Odin

- `core:thread` `Pool` — `pool_init` / `pool_start` / `pool_add_task` /
  `pool_pop_waiting` / `pool_num_outstanding` / `pool_pop_done`
- `Task_Proc` callbacks passing per-job state through `task.data` (`rawptr`)
- `[]Boid` / `[]Vector2` slices + `make` / `delete` manual memory management
- A double-buffered update so every boid sees the same frame state
- `vendor:raylib` bindings and `core:math` / `core:math/rand` / `core:time`

## Notes

It's still `O(n²)` per frame — threading lowers the constant but not the
complexity. The next step would be a **spatial hash grid** to make neighbour
lookups `O(n)`, which composes nicely with the job system (partition the grid
cells across workers instead of raw index ranges).
