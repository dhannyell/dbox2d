package samples

import b2 "github.com/dhannyell/dbox2d"

// setPosX replaces the x coordinate of a position. Positions are float64 in
// large-world builds and scalars otherwise; this keeps the sample code the
// same in every build.
func setPosX(p *b2.Pos, x b2.Q) {
	v := b2.ToVec2(*p)
	v.X = x
	*p = b2.ToPos(v)
}

// setPosY replaces the y coordinate of a position; see setPosX.
func setPosY(p *b2.Pos, y b2.Q) {
	v := b2.ToVec2(*p)
	v.Y = y
	*p = b2.ToPos(v)
}
