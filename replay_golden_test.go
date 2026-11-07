//go:build !dbox2d_fixed

package b2_test

import (
	"fmt"
	"strings"
	"testing"

	. "github.com/dhannyell/dbox2d"
)

// TestGoldenReplayOfUpstreamRecording replays the recording that Box2D
// v3.2.0 wrote, to the end, with every state hash and query reproduced and
// the final transforms bit equal to the ones the C tool printed.
func TestGoldenReplayOfUpstreamRecording(t *testing.T) {
	ResetWorldGenerations()
	data := readGolden(t, ".b2rec")
	if !ValidateRecording(data, 1) {
		t.Fatal("ValidateRecording rejected the upstream recording")
	}

	p := CreateReplay(data, 1)
	if p == nil {
		t.Fatal("CreateReplay failed")
	}
	defer p.Destroy()

	info := p.GetInfo()
	if info.FrameCount != goldenSteps {
		t.Fatalf("frame count %d, want %d", info.FrameCount, goldenSteps)
	}
	for p.StepFrame() {
	}
	if p.HasDiverged() {
		t.Fatalf("the replay diverged at frame %d", p.GetDivergeFrame())
	}
	if p.GetFrame() != goldenSteps || !p.IsAtEnd() {
		t.Fatalf("replay ended at frame %d, atEnd %v", p.GetFrame(), p.IsAtEnd())
	}

	// The final transforms of the first seven bodies, as printed by the C
	// tool.
	var b strings.Builder
	for i := range 7 {
		id := p.GetBodyId(i)
		tf := id.GetTransform()
		x, y := PosBits(tf.P)
		fmt.Fprintf(&b, "body %d %s %s %08x %08x\n", i, hexBits(x), hexBits(y), qBits(tf.Q.Cos), qBits(tf.Q.Sin))
	}
	want := strings.ReplaceAll(string(readGolden(t, ".txt")), "\r", "")
	for _, line := range strings.Split(b.String(), "\n") {
		if line != "" && !strings.Contains(want, line+"\n") {
			t.Errorf("final transform line %q is not in the upstream output", line)
		}
	}
}

// TestGoldenReplaySeekAndRestart checks that seeking back and forth, with
// keyframes, lands on the same state as a straight replay.
func TestGoldenReplaySeekAndRestart(t *testing.T) {
	ResetWorldGenerations()
	data := readGolden(t, ".b2rec")
	p := CreateReplay(data, 1)
	if p == nil {
		t.Fatal("CreateReplay failed")
	}
	defer p.Destroy()
	p.SetKeyframePolicy(0, 8)

	checksumAt := func(frame int) uint64 {
		p.SeekFrame(frame)
		if p.GetFrame() != frame {
			t.Fatalf("seek to %d stopped at %d", frame, p.GetFrame())
		}
		return Checksum(p.GetWorldId())
	}

	forward := map[int]uint64{}
	for _, f := range []int{10, 25, 40, 60} {
		forward[f] = checksumAt(f)
	}
	for _, f := range []int{40, 3, 60, 25, 10, 0, 17} {
		got := checksumAt(f)
		if want, ok := forward[f]; ok && got != want {
			t.Errorf("frame %d: checksum differs after seeking back (%x, want %x)", f, got, want)
		}
	}
	p.Restart()
	if p.GetFrame() != 0 || p.HasDiverged() {
		t.Fatalf("restart left frame %d, diverged %v", p.GetFrame(), p.HasDiverged())
	}
	if got := checksumAt(25); got != forward[25] {
		t.Errorf("frame 25 after restart: %x, want %x", got, forward[25])
	}
	if p.HasDiverged() {
		t.Errorf("diverged after seeking, at frame %d", p.GetDivergeFrame())
	}
}

// TestGoldenQueriesOfFrame checks that the replay exposes the queries that
// the recording holds.
func TestGoldenQueriesOfFrame(t *testing.T) {
	ResetWorldGenerations()
	p := CreateReplay(readGolden(t, ".b2rec"), 1)
	if p == nil {
		t.Fatal("CreateReplay failed")
	}
	defer p.Destroy()

	seen := map[ReplayQueryType]int{}
	for p.StepFrame() {
		for i := range p.GetFrameQueryCount() {
			q := p.GetFrameQuery(i)
			seen[q.Type]++
			for h := range q.HitCount {
				_ = p.GetFrameQueryHit(i, h)
			}
		}
	}
	for _, kind := range []ReplayQueryType{ReplayQueryCastRayClosest, ReplayQueryOverlapAABB, ReplayQueryCastRay} {
		if seen[kind] != 1 {
			t.Errorf("query type %d seen %d times, want 1", kind, seen[kind])
		}
	}
}
