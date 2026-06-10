# boids-poc

A boids flocking simulation in [Odin](https://odin-lang.org), rendered with raylib
(bundled in Odin's `vendor:raylib`). Each boid steers using Craig Reynolds' three
classic rules: **separation**, **alignment**, and **cohesion**.

## Run

```sh
cd boids-poc
odin run . -o:speed
```

## Controls

| Input              | Action                          |
| ------------------ | ------------------------------- |
| Hold left mouse    | Attract the flock to the cursor |
| `1` / `2`          | Separation weight + / -         |
| `3` / `4`          | Alignment weight + / -          |
| `5` / `6`          | Cohesion weight + / -           |
| `H`                | Toggle the help overlay         |

## What this exercises in Odin

- `[]Boid` slices + `make`/`delete` manual memory management
- Structs, `for ... in` with `&` for mutable iteration
- `vendor:raylib` bindings and `core:math` / `core:math/rand`
- A double-buffered update (compute all accelerations, then apply) so every boid
  sees the same frame state

## Notes

It's `O(n²)` — every boid checks every other (300 boids = 90k checks/frame, fine
at 60 FPS). The obvious next step is a spatial grid to make neighbour lookups
`O(n)`.
