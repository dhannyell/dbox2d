# recordgolden

Builds a fixed scene, records 60 steps with API calls in between, then writes
the recording and the final snapshot. `TestGoldenRecordingMatchesUpstream`,
`TestGoldenSnapshotMatchesUpstream` and `TestGoldenReplayOfUpstreamRecording` build the
same scene in Go and compare against the files in `testdata/recording/`.

Build it against the frozen Box2D v3.2.0 checkout, in the single precision or
the double precision build of the reference:

```
cmake -S tools/recordgolden -B build/rec320 -G Ninja -DCMAKE_BUILD_TYPE=Release -DBOX2D_SOURCE_DIR=/path/to/box2d-v3.2.0
cmake --build build/rec320
build/rec320/recordgolden testdata/recording/golden
```

The argument is the file prefix; it writes `<prefix>.b2rec`,
`<prefix>.snapshot` and `<prefix>.txt` (the final transforms). The double
precision files use the prefix `golden_double` and a reference configured with
double positions. The `.txt` has CRLF endings when a Windows build makes it;
the tests strip the carriage returns.
