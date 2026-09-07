# Conformance trace generator

This program generates version 1 collision-function and simulation-scene traces. It links to the frozen Box2D v3.1.1 source tree. The generated files are deterministic inputs for the Go conformance tests.

## Build

The commands, compiler and flags that regenerate the traces are in `testdata/conformance/SOURCE.md`.

The simulation uses one worker and no task callbacks. Each world starts from `b2DefaultWorldDef()`. The benchmark setup functions retain their own sleeping choices. The step size is `1.0f / 60.0f`, with four substeps.

## Regenerate

The program creates `functions` and `scenes` below the output directory. It resets `g_randomSeed` to `RAND_SEED` before every function and scene.

## Common format

Every file starts with this line:

```text
# dbox2d conformance trace v1 kind=<function|scene> name=<name> ref=0aa402e cc=<compiler> flags=<comma-separated-flags>
```

Tokens use one ASCII space as the separator. Lines end with LF, including on Windows. Integers are decimal. Booleans are `0` or `1`. A float is the exact IEEE-754 binary32 bit pattern written as eight lowercase hexadecimal digits. The generator obtains the bits with `memcpy`.

## Function traces

A function file has this shape:

```text
cases <count>
case <index> <inputs> | <outputs>
```

The index starts at zero. The encodings are:

- A vector is `x y`.
- A rotation is `cosine sine`.
- A transform is `position-x position-y cosine sine`.
- A circle is `center-x center-y radius`.
- A capsule is `center1-x center1-y center2-x center2-y radius`.
- A segment is `point1-x point1-y point2-x point2-y`.
- A chain segment is `ghost1-x ghost1-y point1-x point1-y point2-x point2-y ghost2-x ghost2-y chain-id`.
- A polygon is its count, eight vertex slots, eight normal slots, centroid, and radius. Every slot contains two floats. Unused slots are zero.
- A shape proxy is its count, eight point slots, and radius. Unused slots are zero.
- A sweep is `local-center-x local-center-y c1-x c1-y c2-x c2-y q1-c q1-s q2-c q2-s`.
- A simplex cache is `count index-a0 index-a1 index-a2 index-b0 index-b1 index-b2`.
- A manifold is `normal-x normal-y rolling-impulse point-count`. Each active point then adds `point-x point-y anchor-a-x anchor-a-y anchor-b-x anchor-b-y separation id`.
- A distance result is `normal-x normal-y point-a-x point-a-y point-b-x point-b-y distance iterations simplex-count`.
- A time-of-impact result is `state fraction`.
- A hull is its count followed by eight point slots. Unused slots are zero.

The function records use these input and output sequences:

| File | Input | Output |
| --- | --- | --- |
| `collide_circles` | circle A, transform A, circle B, transform B | manifold |
| `collide_capsule_and_circle` | capsule A, transform A, circle B, transform B | manifold |
| `collide_polygon_and_circle` | polygon A, transform A, circle B, transform B | manifold |
| `collide_capsules` | capsule A, transform A, capsule B, transform B | manifold |
| `collide_segment_and_capsule` | segment A, transform A, capsule B, transform B | manifold |
| `collide_polygon_and_capsule` | polygon A, transform A, capsule B, transform B | manifold |
| `collide_polygons` | polygon A, transform A, polygon B, transform B | manifold |
| `collide_segment_and_circle` | segment A, transform A, circle B, transform B | manifold |
| `collide_segment_and_polygon` | segment A, transform A, polygon B, transform B | manifold |
| `collide_chain_segment_and_circle` | chain segment A, transform A, circle B, transform B | manifold |
| `collide_chain_segment_and_capsule` | chain segment A, transform A, capsule B, transform B, zero simplex cache | manifold, updated simplex cache |
| `collide_chain_segment_and_polygon` | chain segment A, transform A, polygon B, transform B, zero simplex cache | manifold, updated simplex cache |
| `shape_distance` | proxy A, proxy B, transform A, transform B, use-radii boolean | distance result |
| `time_of_impact` | proxy A, proxy B, sweep A, sweep B, maximum fraction | time-of-impact result |
| `compute_hull` | count and eight point slots | hull |
| `make_rot` | turns | rotation |
| `atan2` | y, x | radians |

The collision files contain 256 cases each. The chain-segment/capsule and chain-segment/polygon inputs and outputs also contain a simplex cache. `shape_distance` has 512 cases. It passes a zero cache and no simplex-history buffer. `time_of_impact` has 256 cases. `compute_hull` has 128 cases. `make_rot` and `atan2` have 1024 cases each.

Function inputs include contact, separation, deep overlap, zero-radius shapes, short segments and capsules, polygon counts from three through eight, identity rotations, random rotations, and multiple position scales. Hull inputs include collinear and duplicate points. Time-of-impact inputs include crossing paths.

## Scene traces

A scene file starts with:

```text
bodies <count-before-step-1>
steps <total-step-calls>
```

Each call to `b2World_Step` is followed by:

```text
step <one-based-index> <16-digit-hash>
```

A sampled step immediately adds one line for every body in canonical order:

```text
body <zero-based-index> <position-x> <position-y> <rotation-cosine> <rotation-sine>
```

The normal sampling policy writes full dumps at step 1 and at the last step for scenes with at most 5000 bodies; larger scenes retain a final dump only when configured. The checked-in corpus omits the `many_pyramids` final dump, which was the single largest dump. Its per-step hashes remain present. These size reductions leave all other final and eligible step-1 dumps intact.

The canonical order comes from `b2World_OverlapAABB`. The query uses the AABB `[-1000000, -1000000]` to `[1000000, 1000000]` and `b2DefaultQueryFilter()`. The callback maps each reported shape to its body. Body identifiers are deduplicated and sorted by ascending `bodyId.index1`. Static bodies with shapes are included.

The hash is 64-bit FNV-1a. It starts at `14695981039346656037` and uses prime `1099511628211`. For each body it folds position x, position y, rotation cosine, then rotation sine. Each float bit pattern is zero-extended to 64 bits. All eight bytes are folded from least significant to most significant, including the four zero upper bytes. Each byte performs XOR followed by multiplication.

The scene files are `joint_grid`, `large_pyramid`, `many_pyramids`, `rain`, `smash`, `spinner`, `tumbler`, and `falling_hinges`. `StepRain` and `StepSpinner` run before their corresponding world step. `UpdateFallingHinges` does not step the world. The generator calls `b2World_Step`, then calls the update function, then writes one hash. The falling-hinges scene stops when the update returns true or after 2000 steps.

## Produced files

The `functions` directory contains:

- `collide_circles.txt`
- `collide_capsule_and_circle.txt`
- `collide_polygon_and_circle.txt`
- `collide_capsules.txt`
- `collide_segment_and_capsule.txt`
- `collide_polygon_and_capsule.txt`
- `collide_polygons.txt`
- `collide_segment_and_circle.txt`
- `collide_segment_and_polygon.txt`
- `collide_chain_segment_and_circle.txt`
- `collide_chain_segment_and_capsule.txt`
- `collide_chain_segment_and_polygon.txt`
- `shape_distance.txt`
- `time_of_impact.txt`
- `compute_hull.txt`
- `make_rot.txt`
- `atan2.txt`

The `scenes` directory contains one `.txt` file for each scene listed above.
