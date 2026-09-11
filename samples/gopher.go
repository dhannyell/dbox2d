// The Go gopher was designed by Renée French and is licensed under CC BY 4.0.
// This rendition from physics shapes is not in Box2D v3.1.1;

package samples

import (
	"github.com/dhannyell/dbox2d"
)

const (
	gopherBlue  = 0x6AD7E5
	gopherBeige = 0xF6D2A2
	gopherWhite = 0xFFFFFF
	gopherPupil = 0x303030
	gopherNose  = 0x5A4636
)

type gopher struct {
	bodies    [5]dbox2d.BodyId
	isSpawned bool
}

func createGopher(worldId dbox2d.WorldId, position dbox2d.Vec2, scale float64, groupIndex int) gopher {
	q := func(f float64) dbox2d.Q { return FromFloat64(f * scale) }

	bodyDef := dbox2d.DefaultBodyDef()
	bodyDef.Type = dbox2d.DynamicBody
	bodyDef.SleepThreshold = dbox2d.QMustParse("0.1")

	shapeDef := dbox2d.DefaultShapeDef()
	shapeDef.Filter.GroupIndex = -groupIndex

	paintDef := dbox2d.DefaultShapeDef()
	paintDef.Density = dbox2d.QZero()
	paintDef.Filter.CategoryBits = 0
	paintDef.Filter.MaskBits = 0

	type part struct {
		id     dbox2d.BodyId
		ox, oy float64
	}
	newPart := func(ox, oy float64) part {
		bodyDef.Position = position.Add(dbox2d.Vec2{X: q(ox), Y: q(oy)})
		return part{id: dbox2d.CreateBody(worldId, &bodyDef), ox: ox, oy: oy}
	}
	box := func(p part, def *dbox2d.ShapeDef, color uint32, x, y, hw, hh, r float64) {
		center := dbox2d.Vec2{X: q(x - p.ox), Y: q(y - p.oy)}
		poly := dbox2d.MakeOffsetRoundedBox(q(hw), q(hh), center, dbox2d.RotIdentity(), q(r))
		def.Material.CustomColor = color
		dbox2d.CreatePolygonShape(p.id, def, &poly)
	}
	dot := func(p part, def *dbox2d.ShapeDef, color uint32, x, y, r float64) {
		box(p, def, color, x, y, 0.005, 0.005, r)
	}

	body := newPart(0, -0.45)
	dot(body, &paintDef, gopherPupil, -0.19, 0.52, 0.11)
	dot(body, &paintDef, gopherPupil, 0.35, 0.52, 0.11)
	box(body, &paintDef, gopherNose, 0, 0.25, 0.05, 0.02, 0.04)
	box(body, &paintDef, gopherBeige, 0, 0.17, 0.1, 0.03, 0.1)
	box(body, &paintDef, gopherWhite, -0.04, -0.02, 0.03, 0.05, 0.01)
	box(body, &paintDef, gopherWhite, 0.04, -0.02, 0.03, 0.05, 0.01)
	dot(body, &paintDef, gopherWhite, -0.27, 0.55, 0.25)
	dot(body, &paintDef, gopherWhite, 0.27, 0.55, 0.25)
	box(body, &shapeDef, gopherBlue, 0, 0, 0.02, 0.5, 0.5)
	dot(body, &shapeDef, gopherBlue, -0.42, 0.9, 0.1)
	dot(body, &shapeDef, gopherBlue, 0.42, 0.9, 0.1)

	g := gopher{isSpawned: true}
	g.bodies[0] = body.id
	for i, side := range []float64{-1, 1} {
		arm := newPart(side*0.55, 0.05)
		box(arm, &shapeDef, gopherBeige, side*0.55, -0.05, 0.03, 0.1, 0.05)
		foot := newPart(side*0.22, -0.93)
		box(foot, &shapeDef, gopherBeige, side*0.25, -0.98, 0.12, 0.02, 0.05)

		for _, limb := range []struct {
			p     part
			limit float64
		}{{arm, 0.15}, {foot, 0.06}} {
			jd := dbox2d.DefaultRevoluteJointDef()
			jd.BodyIdA = body.id
			jd.BodyIdB = limb.p.id
			jd.LocalAnchorA = dbox2d.Vec2{X: q(limb.p.ox - body.ox), Y: q(limb.p.oy - body.oy)}
			jd.EnableLimit = true
			jd.LowerAngle = FromFloat64(-limb.limit)
			jd.UpperAngle = FromFloat64(limb.limit)
			jd.EnableMotor = true
			jd.MaxMotorTorque = FromFloat64(0.2 * scale * scale * scale)
			jd.EnableSpring = true
			jd.Hertz = dbox2d.QFromInt(5)
			jd.DampingRatio = dbox2d.QHalf()
			jd.DrawSize = q(0.05)
			dbox2d.CreateRevoluteJoint(worldId, &jd)
		}
		g.bodies[1+2*i] = arm.id
		g.bodies[2+2*i] = foot.id
	}
	return g
}

func (g *gopher) destroy() {
	for i, id := range g.bodies {
		dbox2d.DestroyBody(id)
		g.bodies[i] = dbox2d.BodyId{}
	}
	g.isSpawned = false
}
