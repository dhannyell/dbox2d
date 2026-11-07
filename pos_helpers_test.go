package b2

// posX and posY read one coordinate of a position as a scalar. Under
// dbox2d_double a Pos holds float64 coordinates; these helpers keep the
// scalar tests identical in every build mode.
func posX(p Pos) Q { return ToVec2(p).X }

// posY is the Y counterpart of posX.
func posY(p Pos) Q { return ToVec2(p).Y }

// toTransform narrows a world transform to a plain transform for the tests
// that drive the shape-level helpers directly.
func toTransform(t WorldTransform) Transform { return Transform{P: ToVec2(t.P), Q: t.Q} }
