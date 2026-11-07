package samples

import "testing"

// TestRollbackSample runs the reference to sleep, rolls back to a snapshot
// and requires the second run to match.
func TestRollbackSample(t *testing.T) {
	ctx := NewSampleContext()
	s := NewRollback(ctx).(*Rollback)
	defer s.Destroy()

	for i := 0; i < 3000 && s.referenceHash == 0; i++ {
		s.Step()
	}
	if s.referenceHash == 0 || len(s.snapshots) < 2 {
		t.Fatalf("reference run did not finish: hash %x, snapshots %d", s.referenceHash, len(s.snapshots))
	}
	s.rollBack(1)
	for i := 0; i < 3000 && s.result == rollbackRunning; i++ {
		s.Step()
	}
	if s.result != rollbackMatch {
		t.Fatalf("rollback result %d, want match", s.result)
	}
}

// TestReplayViewerSample opens the upstream golden recording and plays it.
func TestReplayViewerSample(t *testing.T) {
	old := ReplayFile
	ReplayFile = "../testdata/recording/golden.b2rec"
	defer func() { ReplayFile = old }()

	ctx := NewSampleContext()
	s := NewReplayViewer(ctx).(*ReplayViewer)
	defer s.Destroy()
	if s.replay == nil {
		t.Skipf("recording not replayable in this build: %s", s.err)
	}
	s.playing = true
	s.loop = false
	for i := 0; i < 1000 && s.playing; i++ {
		s.Step()
	}
	if s.frame != s.info.FrameCount || s.replay.HasDiverged() {
		t.Fatalf("frame %d of %d, diverged %v", s.frame, s.info.FrameCount, s.replay.HasDiverged())
	}
	s.seek(10)
	if s.frame != 10 {
		t.Fatalf("seek landed on %d", s.frame)
	}
}
