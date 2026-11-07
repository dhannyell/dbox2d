// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Ported from samples/sample_replay.cpp of Box2D v3.2.0. The outliner, the
// detail panel and the keyframe popup of the reference viewer are not ported.

package samples

import (
	"strconv"

	b2 "github.com/dhannyell/dbox2d"
)

func init() {
	RegisterSample("Replay", "Viewer", NewReplayViewer)
}

// ReplayFile is the path of the recording that the Replay viewer opens. A
// native host fills it from the -replay flag; the browser host has no file
// system and leaves it empty, so the viewer shows a hint instead.
var ReplayFile string

// ReplayViewer plays a recording written by Box2D v3.2.0 or by this port. It
// owns no world of its own: the world belongs to the replay and is drawn after
// every frame change.
type ReplayViewer struct {
	Base

	replay *b2.Replay
	info   b2.ReplayInfo
	err    string

	frame     int
	speed     float64
	loop      bool
	playing   bool
	accum     float64
	seekFrame int
}

func NewReplayViewer(ctx *SampleContext) Sample {
	s := &ReplayViewer{Base: NewBase(ctx), speed: 1, loop: true}
	// The viewer draws the replay world, so the world Base made is not used.
	b2.DestroyWorld(s.WorldId)
	s.WorldId = b2.WorldId{}

	switch {
	case ReplayFile == "":
		s.err = "no recording: start the native host with -replay <file>"
	default:
		rec := b2.CreateRecordingFromFile(ReplayFile)
		if rec == nil {
			s.err = "cannot read " + ReplayFile
			break
		}
		data := rec.Data()
		rec.Destroy()
		if !b2.ValidateRecording(data, ctx.Settings.WorkerCount) {
			s.err = "not a valid recording: " + ReplayFile
			break
		}
		s.replay = b2.CreateReplay(data, ctx.Settings.WorkerCount)
		if s.replay == nil {
			s.err = "cannot replay " + ReplayFile
			break
		}
		s.info = s.replay.GetInfo()
		s.WorldId = s.replay.GetWorldId()
		s.frameCamera()
	}
	// The reference viewer starts paused.
	ctx.Settings.Pause = true
	return s
}

// frameCamera centers the camera on the recorded bounds.
func (s *ReplayViewer) frameCamera() {
	ctx := s.Context
	if ctx.Settings.Restart {
		return
	}
	b := s.info.Bounds
	w := ToFloat64(b.UpperBound.X.Sub(b.LowerBound.X))
	h := ToFloat64(b.UpperBound.Y.Sub(b.LowerBound.Y))
	if w <= 0 && h <= 0 {
		ctx.Camera.Center = Vec2f{X: 0, Y: 10}
		ctx.Camera.Zoom = 20
		return
	}
	ratio := float64(ctx.Camera.Width) / float64(ctx.Camera.Height)
	ctx.Camera.Center = Vec2f{
		X: ToFloat64(b.LowerBound.X) + w/2,
		Y: ToFloat64(b.LowerBound.Y) + h/2,
	}
	ctx.Camera.Zoom = 1.2 * max(h/2, w/2/ratio, 1)
}

func (s *ReplayViewer) advance() {
	if s.replay == nil {
		return
	}
	if !s.replay.StepFrame() && s.replay.IsAtEnd() {
		if s.loop {
			s.replay.Restart()
		} else {
			s.playing = false
		}
	}
	s.WorldId = s.replay.GetWorldId()
	s.frame = s.replay.GetFrame()
}

func (s *ReplayViewer) seek(frame int) {
	if s.replay == nil {
		return
	}
	s.replay.SeekFrame(frame)
	s.WorldId = s.replay.GetWorldId()
	s.frame = s.replay.GetFrame()
}

func (s *ReplayViewer) UpdateGui() {
	gui := s.Context.Gui
	if !gui.Begin("Replay", 10, 300, 260, 260) {
		gui.End()
		return
	}
	if s.replay == nil {
		gui.Text(s.err)
		gui.End()
		return
	}
	label := "Play"
	if s.playing {
		label = "Pause"
	}
	if gui.Button(label) {
		s.playing = !s.playing
	}
	if gui.Button("Step") {
		s.playing = false
		s.advance()
	}
	if gui.Button("Step Back") {
		s.playing = false
		s.seek(s.frame - 1)
	}
	if gui.Button("Restart") {
		s.replay.Restart()
		s.WorldId = s.replay.GetWorldId()
		s.frame = 0
	}
	gui.SliderFloat("Speed", &s.speed, 0.1, 4)
	gui.Checkbox("Loop", &s.loop)
	s.seekFrame = s.frame
	if gui.SliderInt("Frame", &s.seekFrame, 0, max(s.info.FrameCount, 1)) {
		s.playing = false
		s.seek(s.seekFrame)
	}
	gui.Text("frame " + strconv.Itoa(s.frame) + " / " + strconv.Itoa(s.info.FrameCount))
	if s.replay.HasDiverged() {
		gui.Text("DIVERGED at frame " + strconv.Itoa(s.replay.GetDivergeFrame()))
	} else {
		gui.Text("no divergence")
	}
	gui.End()
}

func (s *ReplayViewer) Keyboard(key Key) {
	switch key {
	case KeySpace:
		s.playing = !s.playing
	case KeyPeriod:
		s.playing = false
		s.advance()
	case KeyComma:
		s.playing = false
		step := 1
		if s.Context.IsKeyDown != nil && s.Context.IsKeyDown(KeyLeftShift) {
			step = 5
		}
		s.seek(s.frame - step)
	}
}

func (s *ReplayViewer) Step() {
	if s.replay == nil {
		s.DrawTextLine("%s", s.err)
		return
	}

	if s.playing {
		s.accum += s.speed
		for s.accum >= 1 && s.playing {
			s.accum--
			s.advance()
		}
	}

	s.applyDrawSettings()
	draw := &s.Context.Draw
	s.WorldId.Draw(draw)
	s.replay.DrawFrameQueries(draw, -1)

	s.DrawTextLine("frame %d / %d", s.frame, s.info.FrameCount)
	if s.replay.HasDiverged() {
		s.DrawTextLine("DIVERGED at frame %d", s.replay.GetDivergeFrame())
	}
}

// Mouse input does nothing: there is no world of the viewer's own to grab.
func (s *ReplayViewer) MouseDown(b2.Vec2, MouseButton, Modifier) {}
func (s *ReplayViewer) MouseUp(b2.Vec2, MouseButton)             {}
func (s *ReplayViewer) MouseMove(b2.Vec2)                        {}

func (s *ReplayViewer) Destroy() {
	if s.replay != nil {
		s.replay.Destroy()
		s.replay = nil
	}
}
