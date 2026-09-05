// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// The reference draws text through ImGui, not a rasterized atlas; this port
// needs its own glyph atlas because a WGSL host has no ImGui text layer.

package draw

import (
	"image"
	"image/color"
	imagedraw "image/draw"

	_ "embed"

	"golang.org/x/image/font"
	"golang.org/x/image/font/opentype"
	"golang.org/x/image/math/fixed"
)

// whiteBlockSize is the side, in pixels, of the opaque texel block reserved
// at the top-left of the atlas so the text pipeline also draws solid UI
// quads, with no second pipeline.
const whiteBlockSize = 2

//go:embed data/droid_sans.ttf
var droidSansTTF []byte

// firstGlyphRune and lastGlyphRune bound the ASCII range the atlas
// rasterizes; anything outside it maps to '?'.
const (
	firstGlyphRune = ' '
	lastGlyphRune  = '~'
	glyphCount     = lastGlyphRune - firstGlyphRune + 1

	atlasWidth = 512
)

// TextVertex is one vertex of the text pipeline's per-vertex buffer.
type TextVertex struct {
	X, Y, U, V float32
	Color      RGBA8
}

// glyph is one atlas entry: texture coordinates plus the metrics needed to
// place it relative to the pen position.
type glyph struct {
	U0, V0, U1, V1                    float32
	W, H, BearingX, BearingY, Advance int
}

// Atlas is the rasterized font: one alpha texture and the glyph table.
type Atlas struct {
	Width, Height      int
	Pixels             []uint8 // r8unorm, row-major
	Ascent, LineHeight int
	// WhiteU, WhiteV address the opaque texel block reserved at the atlas
	// origin, so UI rects sample it and share the text pipeline.
	WhiteU, WhiteV float32
	glyphs         [glyphCount]glyph
}

// NewAtlas rasterizes droid_sans.ttf at the given pixel size (the
// reference uses 14 for the regular font).
func NewAtlas(pixelSize int) (*Atlas, error) {
	f, err := opentype.Parse(droidSansTTF)
	if err != nil {
		return nil, err
	}
	face, err := opentype.NewFace(f, &opentype.FaceOptions{
		Size:    float64(pixelSize),
		DPI:     72,
		Hinting: font.HintingFull,
	})
	if err != nil {
		return nil, err
	}
	defer func() { _ = face.Close() }()

	type placed struct {
		r       rune
		dr      image.Rectangle
		advance int
		x, y    int
	}
	placedGlyphs := make([]placed, 0, glyphCount)
	for i := range glyphCount {
		r := rune(firstGlyphRune + i)
		dr, _, _, advance, ok := face.Glyph(fixed.P(0, 0), r)
		if !ok {
			continue
		}
		placedGlyphs = append(placedGlyphs, placed{r: r, dr: dr, advance: advance.Round()})
	}

	// Shelf-pack the glyphs left to right, wrapping rows at atlasWidth. The
	// one-pixel gutter keeps a linear sampler from bleeding neighbours.
	const gutter = 1
	whiteX, whiteY := 0, 0
	x, y, rowHeight := whiteBlockSize+gutter, 0, whiteBlockSize
	for i := range placedGlyphs {
		w := placedGlyphs[i].dr.Dx()
		if x+w > atlasWidth {
			x = 0
			y += rowHeight + gutter
			rowHeight = 0
		}
		placedGlyphs[i].x, placedGlyphs[i].y = x, y
		x += w + gutter
		if h := placedGlyphs[i].dr.Dy(); h > rowHeight {
			rowHeight = h
		}
	}
	height := nextPowerOfTwo(y + rowHeight)

	img := image.NewAlpha(image.Rect(0, 0, atlasWidth, height))
	for wy := range whiteBlockSize {
		for wx := range whiteBlockSize {
			img.SetAlpha(whiteX+wx, whiteY+wy, color.Alpha{A: 255})
		}
	}
	var glyphs [glyphCount]glyph
	for _, g := range placedGlyphs {
		// The face reuses one mask buffer across Glyph calls, so each
		// glyph is rasterized again right before it is copied.
		_, mask, maskp, _, _ := face.Glyph(fixed.P(0, 0), g.r)
		dst := image.Rect(g.x, g.y, g.x+g.dr.Dx(), g.y+g.dr.Dy())
		imagedraw.Draw(img, dst, mask, maskp, imagedraw.Src)

		glyphs[g.r-firstGlyphRune] = glyph{
			U0:       float32(g.x) / float32(atlasWidth),
			V0:       float32(g.y) / float32(height),
			U1:       float32(g.x+g.dr.Dx()) / float32(atlasWidth),
			V1:       float32(g.y+g.dr.Dy()) / float32(height),
			W:        g.dr.Dx(),
			H:        g.dr.Dy(),
			BearingX: g.dr.Min.X,
			BearingY: g.dr.Min.Y,
			Advance:  g.advance,
		}
	}

	metrics := face.Metrics()
	return &Atlas{
		Width:      atlasWidth,
		Height:     height,
		Pixels:     img.Pix,
		Ascent:     metrics.Ascent.Round(),
		LineHeight: metrics.Height.Round(),
		WhiteU:     (float32(whiteX) + 0.5*whiteBlockSize) / float32(atlasWidth),
		WhiteV:     (float32(whiteY) + 0.5*whiteBlockSize) / float32(height),
		glyphs:     glyphs,
	}, nil
}

func nextPowerOfTwo(n int) int {
	p := 1
	for p < n {
		p *= 2
	}
	return p
}

// glyphFor returns the atlas entry for r, or '?' outside the rasterized range.
func (a *Atlas) glyphFor(r rune) glyph {
	if r < firstGlyphRune || r > lastGlyphRune {
		r = '?'
	}
	return a.glyphs[r-firstGlyphRune]
}

// TextWidth sums the glyph advances of s, in pixels; the microui layer
// calls it to lay out controls.
func (a *Atlas) TextWidth(s string) int {
	width := 0
	for _, r := range s {
		width += a.glyphFor(r).Advance
	}
	return width
}

// TextHeight returns the line height, in pixels.
func (a *Atlas) TextHeight() int { return a.LineHeight }

// AppendRectQuad appends one solid-colour quad sampling the atlas's white
// texel, for the text pipeline to also draw UI rects.
func (a *Atlas) AppendRectQuad(out []TextVertex, x, y, w, h float32, c RGBA8) []TextVertex {
	x0, y0, x1, y1 := x, y, x+w, y+h
	u, v := a.WhiteU, a.WhiteV
	return append(out,
		TextVertex{X: x0, Y: y0, U: u, V: v, Color: c},
		TextVertex{X: x1, Y: y0, U: u, V: v, Color: c},
		TextVertex{X: x1, Y: y1, U: u, V: v, Color: c},
		TextVertex{X: x0, Y: y0, U: u, V: v, Color: c},
		TextVertex{X: x1, Y: y1, U: u, V: v, Color: c},
		TextVertex{X: x0, Y: y1, U: u, V: v, Color: c},
	)
}

// AppendQuads appends six TextVertex per glyph of item to out, positioned
// in pixels with y down and item.Y at the text's top.
func (a *Atlas) AppendQuads(out []TextVertex, item TextItem) []TextVertex {
	penX := item.X
	baseline := item.Y + float32(a.Ascent)
	for _, r := range item.Text {
		g := a.glyphFor(r)
		x0 := penX + float32(g.BearingX)
		y0 := baseline + float32(g.BearingY)
		x1 := x0 + float32(g.W)
		y1 := y0 + float32(g.H)

		out = append(out,
			TextVertex{X: x0, Y: y0, U: g.U0, V: g.V0, Color: item.Color},
			TextVertex{X: x1, Y: y0, U: g.U1, V: g.V0, Color: item.Color},
			TextVertex{X: x1, Y: y1, U: g.U1, V: g.V1, Color: item.Color},
			TextVertex{X: x0, Y: y0, U: g.U0, V: g.V0, Color: item.Color},
			TextVertex{X: x1, Y: y1, U: g.U1, V: g.V1, Color: item.Color},
			TextVertex{X: x0, Y: y1, U: g.U0, V: g.V1, Color: item.Color},
		)
		penX += float32(g.Advance)
	}
	return out
}
