# drawgolden

Prints every debug draw callback of a fixed scene, as `testdata/draw_golden.txt`
expects. `TestDrawMatchesGolden` builds the same scene in Go and compares the
calls in order, with a 1e-3 budget on every number.

Build it against the frozen Box2D v3.2.0 checkout with the flags of the
conformance tool:

```
cmake -S tools/drawgolden -B build/draw320 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=/path/to/box2d-v3.2.0
cmake --build build/draw320
build/draw320/drawgolden | tr -d '\r' > testdata/draw_golden.txt
```

The golden is stored with LF endings; the `tr` strips the carriage returns a
Windows build prints.
