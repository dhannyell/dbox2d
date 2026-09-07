package app_test

import (
	"strings"
	"testing"

	"github.com/dhannyell/dbox2d/samples"
	"github.com/dhannyell/dbox2d/samples/internal/app"
	"github.com/dhannyell/dbox2d/samples/internal/draw"
)

// fakeMeasurer avoids rasterizing a real font just to lay out the tools
// window in a headless test.
type fakeMeasurer struct{}

func (fakeMeasurer) TextWidth(s string) int { return 7 * len(s) }
func (fakeMeasurer) TextHeight() int        { return 14 }

func TestAppRunsTheTumblerHeadless(t *testing.T) {
	a := app.New(fakeMeasurer{})
	a.Resize(1920, 1080)

	var solidPolygons int
	var sawControls bool
	for range 30 {
		b, cmds := a.Frame(1.0 / 60)
		solidPolygons = len(b.SolidPolygons)
		if len(cmds) == 0 {
			t.Fatalf("frame produced no UI commands")
		}
		for _, c := range cmds {
			if c.Text == "Controls" {
				sawControls = true
			}
		}
	}
	if solidPolygons <= 2000 {
		t.Fatalf("got %d solid polygons on the last frame, want > 2000", solidPolygons)
	}
	if !sawControls {
		t.Fatalf("no UI command carried the \"Controls\" label")
	}

	a.KeyDown(samples.KeyP, 0)
	if !a.Settings().Pause {
		t.Fatalf("KeyP did not pause")
	}

	before := a.Settings().SampleIndex
	a.KeyDown(samples.KeyRightBracket, 0)
	a.Frame(1.0 / 60)
	if a.Settings().SampleIndex == before {
		t.Fatalf("KeyRightBracket did not change SampleIndex")
	}

	a.KeyDown(samples.KeyR, 0)
	if !a.Settings().Restart {
		t.Fatalf("KeyR did not set Restart")
	}
	a.Frame(1.0 / 60)
	if a.Settings().Restart {
		t.Fatalf("Restart was not cleared by the next frame")
	}
}

// The bottom-left overlay reports the host's frame time and follows the
// UI toggle, as in the reference.
func TestFrameOverlayFollowsTheUIToggle(t *testing.T) {
	a := app.New(fakeMeasurer{})
	a.Resize(1920, 1080)

	overlay := func(b *draw.Batches) string {
		for _, item := range b.Text {
			if strings.Contains(item.Text, " ms - step ") {
				return item.Text
			}
		}
		return ""
	}

	b, _ := a.Frame(0.0125)
	line := overlay(b)
	if !strings.HasPrefix(line, "12.5 ms - step 1 - camera (") {
		t.Fatalf("overlay = %q, want the 12.5 ms line for step 1", line)
	}

	a.KeyDown(samples.KeyTab, 0)
	b, _ = a.Frame(0.0125)
	if line := overlay(b); line != "" {
		t.Fatalf("overlay %q still drawn with the UI hidden", line)
	}
}
