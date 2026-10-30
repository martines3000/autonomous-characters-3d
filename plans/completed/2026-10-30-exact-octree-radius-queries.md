# Exact octree radius queries

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Octree mode is meant to be a faster way to compute the same flocking as naive mode. Today it is not the same: `Octree::find_neighbors` returns only the points stored in the single leaf that contains the vehicle, so neighbors just across a cell boundary are missed, and vehicles near cell edges flock differently from naive mode. Coincident points also make the tree subdivide one level deeper on every insert, without bound.

After this work, octree mode finds exactly the neighbors naive mode finds (every vehicle within the largest flocking radius), so both modes produce the same steering up to floating-point summation order, and subdivision depth is bounded. This makes the later benchmark comparison between the modes meaningful.

## Progress

- [x] (2026-10-30 14:40Z) Plan drafted; findings F02 and F04 confirmed in current `src/octree.rs`.
- [x] (2026-10-30 15:05Z) Milestone 1: `Octree::query_radius`, coordinate-based child selection, `MAX_DEPTH` 16; brute-force reference, thin-root and coincident-point tests.
- [x] (2026-10-30 15:05Z) Milestone 2: `flock` builds the root from vehicle positions and queries the largest flocking radius; naive/octree equivalence test on 384 seeded vehicles. The equivalence test failed on the old single-leaf lookup (vehicle 2: naive x acceleration -14.22, octree 18.03) and passes now.
- [x] (2026-10-30 15:20Z) Milestone 3: diff reviewed; added a `SteeringQuery` type alias to clear a new Clippy warning; gate passed.
- [x] (2026-10-30 15:25Z) Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: the "closest child center" insertion rule is not reliable. In a thin box (one axis much longer than the others) squared center distances round to ties across a split plane, and a point lands in a child whose bounds do not contain it, which breaks bounds-based pruning.
  Evidence: `thin_root_keeps_points_in_containing_children` (root ±1 × ±1e6 × ±1).
- Observation: returning query results in insertion order makes octree and naive mode sum neighbors in the same order, so the equivalence test passes at a 1e-4 tolerance rather than needing a loose one.
  Evidence: `octree_flocking_matches_naive_on_seeded_world`.

## Decision Log

- Decision: build the octree root from the bounding box of the current vehicle positions instead of `WORLD_SIZE`.
  Rationale: walls are soft (vehicles overshoot by up to 6 units) and seeking ignores walls, so vehicles can sit outside a fixed root. Insertion then assigns them to the nearest child, outside that child's bounds, and a radius query that prunes by node bounds would miss them. Covering every point keeps "a point lies inside its node's bounds" true, which is what makes pruning exact.
  Date: 2026-10-30.
- Decision: query radius is the largest of the three flocking distances (times `vehicle_size`); `flocking_force` keeps applying its own strict per-behavior thresholds.
  Rationale: one query per vehicle; the query returns a superset (distance ≤ radius) and the existing thresholds pick exactly what naive mode picks.
  Date: 2026-10-30.
- Decision: bound subdivision with a maximum depth; at that depth a leaf keeps accepting points beyond capacity.
  Rationale: the simplest guard against coincident or near-coincident points; exactness does not depend on leaf size.
  Date: 2026-10-30.
- Decision: keep naive mode as the default, keep the per-vehicle result `Vec`, keep rebuilding the tree every frame.
  Rationale: default and performance choices need the seeded benchmark (future task 3) and profiling (task 4). This task is about correctness.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption (wrong; see Surprises): insertion's "closest child center" rule puts every point inside the root into the child octant whose bounds contain it (boundary points into one of the touching children).
  Why reasonable: child centers sit at the quarter points of each axis, so the nearest center is the containing octant.
  Validation: the reference test includes points on split planes and on the root boundary.
  If wrong: assign children by coordinate comparison against the node center instead. Done.

## Outcomes & Retrospective

Delivered: octree mode now finds every neighbor within the largest flocking radius and produces the same accelerations as naive mode (tested on 384 seeded vehicles with clusters and vehicles outside the walls). Subdivision stops at depth 16, so 5,000 coincident points create 129 nodes instead of a path thousands of levels deep. Findings F02 and F04 are resolved. 18 tests pass; Clippy warnings fell from 14 to 9.

Not done, by design: performance is unmeasured. The query visits more nodes than the old single-leaf lookup, sorts its results and still allocates a `Vec` per vehicle, and the tree is rebuilt every frame. Whether octree mode beats naive mode, and at what count, is for the seeded benchmark (future task 3). The default stays naive.

Residual risk: the query and `flocking_force` compute squared distances in different operation orders, so a neighbor lying within one rounding step of the radius could be excluded by the query but counted by naive mode. This only affects vehicles at the exact radius boundary and only for one frame; not worth extra code.

Lessons: an oracle test against brute force found the insertion bug that a reading of the code had assumed away in this plan's own assumption.

## Context and Orientation

- `src/octree.rs`: `Octree<T>` stores nodes and points in append-only vectors. `insert` walks from the root choosing the child with the closest center, appends to a leaf, and calls `subdivide` when the leaf exceeds `max_points`. `subdivide` creates eight children and redistributes the leaf's points by closest center. `find_neighbors` descends to one leaf and returns its points. `OctreeNode::bounds` is `(center, min, max)`. One test covers indices surviving subdivision.
- `src/vehicle.rs`: `flock` (octree branch) builds a tree each frame with root bounds ±`WORLD_SIZE` and `max_points = state.octree_size` (UI slider 2–500, default 100), inserts every vehicle as `OctreeData { entity, transform, velocity }`, then per vehicle calls `find_neighbors` and passes the neighbors (minus itself) to `flocking_force`. The naive branch passes all other vehicles. `flocking_force` filters by strict `distance² < radius²` per behavior.
- `src/main.rs`: `test_octree_visualization` (unused debug system) reads `nodes`, `points` and `bounds`; keep it compiling.

## Plan of Work

1. In `src/octree.rs`, add a radius query that visits nodes whose bounds are within the radius of the query point (squared distance from point to box) and returns stored points within the radius. Remove `find_neighbors`. Add a depth limit to subdivision.
2. Tests in `src/octree.rs`: seeded random point sets (uniform, clustered, on split planes, coincident) and many query points and radii; the query's result set equals a brute-force scan. Coincident insertions keep node count bounded.
3. In `src/vehicle.rs`, compute the root bounds from vehicle positions, query with the largest flocking radius, and keep everything else as is.
4. Test in `src/vehicle.rs`: on one seeded world, every vehicle's acceleration after `flock` matches between naive and octree modes within a small tolerance, with a small `octree_size` so the tree is deep.
5. Gate.

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`

## Validation and Acceptance

- Reference test: radius query equals brute force on every generated case.
- Equivalence test: naive and octree accelerations agree per vehicle (absolute tolerance 1e-4, or relative where forces are large) on a seeded world of a few hundred vehicles, including vehicles outside `WORLD_SIZE`. The test fails with the old single-leaf lookup.
- Inserting thousands of coincident points creates a bounded number of nodes.
- All existing tests pass; startup smoke passes.

## Idempotence and Recovery

Changes are confined to `src/octree.rs` and the octree branch of `flock`. Revert the commit to restore single-leaf behavior.

## Artifacts and Notes

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`): `cargo fmt --all -- --check` clean; `cargo test --release --locked` 18 passed; `cargo build --release --locked` with 1 known warning (`benchmark` unused; the old `is_point_in_bounds` warning is gone with the function); `python3 scripts/smoke-native.py` PASS; `git diff --check` clean; Clippy 9 warnings, none in changed code.

## Interfaces and Dependencies

- `Octree::find_neighbors(&self, &Point) -> Vec<&T>` removed; `Octree::query_radius(&self, point: &Point, radius: f32) -> Vec<&T>` added (inclusive radius, results in insertion order). Points must lie within the root bounds.
- `Octree::is_point_in_bounds` removed; private `child_for_point` added. `const MAX_DEPTH: usize = 16`. `OctreeNode` is unchanged (depth is tracked during insertion).
- `src/vehicle.rs`: `type SteeringQuery` alias for the steering query shared by `movement`, `flock` and a test. No new crates.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded implementation, insertion-rule discovery, gate; closed.
