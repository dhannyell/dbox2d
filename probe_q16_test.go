package dbox2d

import (
	"math"
	"math/big"
	"math/bits"
	"math/rand"
	"testing"

	"github.com/dhannyell/fixed"
)

// This file is a probe, not library code. It mirrors the contact stages
// and the two integrations over a narrow working format: int32 with
// fracBits of fraction. The scalar products and the effective mass sums
// accumulate in int64 on the same grid, which is Q48.16 when fracBits is
// 16. With 16 bits the format reproduces fixed.Q16 and fixed.Q48 bit for
// bit; TestProbeFormatMatchesQ16AndQ48 proves it. With 12 bits it is a
// Q20.12. Positions and rotations stay in Q32 and cross the boundary in
// the prepare and in the finalize. Nothing here enters the library.
//
// The lanes row keeps the state in Q32 too: velocities, position deltas
// and the impulse sums never leave Q32. The contact stages read them
// into the working format, rounded to nearest, compute one constraint,
// and write back only the delta. The integrations run in Q32. That row
// measures the working format as a lane format, not as a state format.

// probeFormat is the working format: fracBits of fraction in an int32,
// with its own saturation counter.
type probeFormat struct {
	fracBits    uint
	one         int32
	saturations int

	// accSaturations counts the saturations of the accumulator, which
	// follow the Q48 rules: a sum or a quotient outside int64 clamps.
	accSaturations int

	// nearest rounds every product to the nearest grid value instead of
	// the floor of Q16.Mul. A diagnosis switch, off in the plan rows.
	nearest bool
}

// product floors or rounds a raw product to the grid.
func (f *probeFormat) product(a, b int32) int64 {
	p := int64(a) * int64(b)
	if f.nearest {
		p += 1 << (f.fracBits - 1)
	}
	return p >> f.fracBits
}

func newProbeFormat(fracBits uint) *probeFormat {
	return &probeFormat{fracBits: fracBits, one: 1 << fracBits}
}

func (f *probeFormat) sat(v int64) int32 {
	if v > math.MaxInt32 {
		f.saturations++
		return math.MaxInt32
	}
	if v < math.MinInt32 {
		f.saturations++
		return math.MinInt32
	}
	return int32(v)
}

func (f *probeFormat) add(a, b int32) int32 { return f.sat(int64(a) + int64(b)) }
func (f *probeFormat) sub(a, b int32) int32 { return f.sat(int64(a) - int64(b)) }
func (f *probeFormat) neg(a int32) int32    { return f.sat(-int64(a)) }

// mul floors the product like Q16.Mul.
func (f *probeFormat) mul(a, b int32) int32 {
	return f.sat(f.product(a, b))
}

// div truncates toward zero like Q16.Div.
func (f *probeFormat) div(a, b int32) int32 {
	return f.sat((int64(a) << f.fracBits) / int64(b))
}

func (f *probeFormat) abs(a int32) int32 {
	if a >= 0 {
		return a
	}
	return f.neg(a)
}

func (f *probeFormat) max(a, b int32) int32 {
	if a > b {
		return a
	}
	return b
}

func (f *probeFormat) clamp(v, lo, hi int32) int32 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// isqrt64 returns floor(sqrt(n)). The float seed is corrected to the
// exact root, so the result does not depend on the float rounding.
func isqrt64(n uint64) uint64 {
	x := uint64(math.Sqrt(float64(n)))
	for x > 0 && x*x > n {
		x--
	}
	for (x+1)*(x+1) <= n {
		x++
	}
	return x
}

// sqrt floors the root like Q16.Sqrt.
func (f *probeFormat) sqrt(a int32) int32 {
	if a < 0 {
		panic("probe: square root of a negative value")
	}
	return int32(isqrt64(uint64(a) << f.fracBits))
}

// The accumulator is an int64 on the working grid: Q48.16 when fracBits
// is 16. widen is Q16.ToQ48, narrow is Q48.ToQ16, mulAdd is Q48.MulAdd16,
// mulSub is the Q48.Sub of one floored product, and divAcc is Q48.Div.
// The sums and the quotient saturate and count like Q48.

func (f *probeFormat) widen(a int32) int64    { return int64(a) }
func (f *probeFormat) narrow(acc int64) int32 { return f.sat(acc) }

// addAcc mirrors Q48.Add; subAcc mirrors Q48.Sub.
func (f *probeFormat) addAcc(a, b int64) int64 {
	res := a + b
	if (a >= 0) == (b >= 0) && (res >= 0) != (a >= 0) {
		f.accSaturations++
		return math.MaxInt64 ^ (a >> 63)
	}
	return res
}

func (f *probeFormat) subAcc(a, b int64) int64 {
	res := a - b
	if (a >= 0) != (b >= 0) && (res >= 0) != (a >= 0) {
		f.accSaturations++
		return math.MaxInt64 ^ (a >> 63)
	}
	return res
}

func (f *probeFormat) mulAdd(acc int64, a, b int32) int64 {
	return f.addAcc(acc, f.product(a, b))
}

func (f *probeFormat) mulSub(acc int64, a, b int32) int64 {
	return f.subAcc(acc, f.product(a, b))
}

// divAcc mirrors Q48.Div: the numerator widens to 128 bits, the quotient
// truncates toward zero and saturates.
func (f *probeFormat) divAcc(n, d int64) int64 {
	if d == 0 {
		panic("probe: division by zero")
	}
	neg := (n < 0) != (d < 0)
	un, ud := magnitude64(n), magnitude64(d)
	hi, lo := un>>(64-f.fracBits), un<<f.fracBits
	if hi >= ud {
		f.accSaturations++
		if neg {
			return math.MinInt64
		}
		return math.MaxInt64
	}
	quo, _ := bits.Div64(hi, lo, ud)
	if neg {
		if quo > 1<<63 {
			f.accSaturations++
			return math.MinInt64
		}
		return -int64(quo)
	}
	if quo > math.MaxInt64 {
		f.accSaturations++
		return math.MaxInt64
	}
	return int64(quo)
}

func magnitude64(v int64) uint64 {
	if v < 0 {
		return uint64(-v)
	}
	return uint64(v)
}

// sqrtAcc floors the root of an accumulator like Q48.Sqrt. The root of
// the widened radicand fits int64, so it never saturates; the wide
// radicand takes the slow path.
func (f *probeFormat) sqrtAcc(acc int64) int64 {
	if acc < 0 {
		panic("probe: square root of a negative value")
	}
	if uint64(acc) < 1<<(64-f.fracBits) {
		return int64(isqrt64(uint64(acc) << f.fracBits))
	}
	r := new(big.Int).Lsh(big.NewInt(acc), f.fracBits)
	return r.Sqrt(r).Int64()
}

// fromQ floors a Q32 to the working grid like Q32.ToQ16; toQ widens like
// Q16.ToQ32. fromQNearest rounds to the nearest grid value; the lanes row
// reads the Q32 state through it. No library operation does this.
func (f *probeFormat) fromQ(q Q) int32 { return f.sat(q.Raw() >> (32 - f.fracBits)) }
func (f *probeFormat) fromQNearest(q Q) int32 {
	if !f.nearest {
		return f.fromQ(q)
	}
	return f.sat((q.Raw() + 1<<(31-f.fracBits)) >> (32 - f.fracBits))
}
func (f *probeFormat) fromVecNearest(v Vec2) pv { return pv{f.fromQNearest(v.X), f.fromQNearest(v.Y)} }
func (f *probeFormat) toQ(a int32) Q            { return fixed.Q32FromRaw(int64(a) << (32 - f.fracBits)) }

// pv is a vector in the working format.
type pv struct{ x, y int32 }

func (f *probeFormat) fromVec(v Vec2) pv { return pv{f.fromQ(v.X), f.fromQ(v.Y)} }
func (f *probeFormat) toVec(v pv) Vec2   { return Vec2{X: f.toQ(v.x), Y: f.toQ(v.y)} }

func (f *probeFormat) vadd(a, b pv) pv { return pv{f.add(a.x, b.x), f.add(a.y, b.y)} }
func (f *probeFormat) vsub(a, b pv) pv { return pv{f.sub(a.x, b.x), f.sub(a.y, b.y)} }
func (f *probeFormat) vscale(v pv, s int32) pv {
	return pv{f.mul(v.x, s), f.mul(v.y, s)}
}

// vmulAdd mirrors MulAdd: a + s * b. vmulSub mirrors MulSub.
func (f *probeFormat) vmulAdd(a pv, s int32, b pv) pv {
	return pv{f.add(a.x, f.mul(s, b.x)), f.add(a.y, f.mul(s, b.y))}
}

func (f *probeFormat) vmulSub(a pv, s int32, b pv) pv {
	return pv{f.sub(a.x, f.mul(s, b.x)), f.sub(a.y, f.mul(s, b.y))}
}

// dot and cross accumulate the two products and narrow once.
func (f *probeFormat) dot(a, b pv) int32 {
	return f.narrow(f.mulAdd(f.mulAdd(0, a.x, b.x), a.y, b.y))
}

func (f *probeFormat) cross(a, b pv) int32 {
	return f.narrow(f.mulSub(f.mulAdd(0, a.x, b.y), a.y, b.x))
}

// crossSV mirrors CrossSV: (-s * v.y, s * v.x).
func (f *probeFormat) crossSV(s int32, v pv) pv {
	return pv{f.mul(f.neg(s), v.y), f.mul(s, v.x)}
}

// rotate mirrors Rot.Apply with the working products.
func (f *probeFormat) rotate(c, s int32, v pv) pv {
	return pv{f.sub(f.mul(c, v.x), f.mul(s, v.y)), f.add(f.mul(s, v.x), f.mul(c, v.y))}
}

// probeSoftness mirrors softness.
type probeSoftness struct{ biasRate, massScale, impulseScale int32 }

// makeSoft mirrors makeSoft with the working divisions.
func (f *probeFormat) makeSoft(hertz, zeta, h, tau int32) probeSoftness {
	if hertz == 0 {
		return probeSoftness{}
	}
	omega := f.mul(tau, hertz)
	a1 := f.add(f.add(zeta, zeta), f.mul(h, omega))
	a2 := f.mul(f.mul(h, omega), a1)
	return probeSoftness{
		biasRate:     f.div(omega, a1),
		massScale:    f.div(a2, f.add(f.one, a2)),
		impulseScale: f.div(f.one, f.add(f.one, a2)),
	}
}

// probeSim keeps the Q32 side of a body: the pose. The masses sit in the
// working format because only the solver reads them.
type probeSim struct {
	transform     Transform
	center        Vec2
	localCenter   Vec2
	maxExtent     Q
	invMass       int32
	invInertia    int32
	isSpeedCapped bool
}

// probeState mirrors bodyState in the working format. The lanes row uses
// the Q32 fields instead and leaves the working fields at zero.
type probeState struct {
	v        pv
	w        int32
	dp       pv
	dqc, dqs int32

	v32  Vec2
	w32  Q
	dp32 Vec2
	dq32 Rot
}

// probeShape keeps the Q32 bounds of one box.
type probeShape struct{ aabb, fatAABB AABB }

// probeContact keeps the Q32 manifold and the working impulses. The lanes
// row keeps the impulses in the Q32 fields.
type probeContact struct {
	indexA, indexB int
	manifold       Manifold
	nimp, timp     [2]int32
	tnimp, nvel    [2]int32
	rolling        int32
	friction       int32
	restitution    int32
	touching       bool

	nimp32, timp32, tnimp32 [2]Q
}

type probePoint struct {
	rA, rB             pv
	baseSeparation     int32
	relativeVelocity   int32
	normalImpulse      int32
	tangentImpulse     int32
	totalNormalImpulse int32
	normalMass         int32
	tangentMass        int32

	ni32, ti32, tni32 Q
}

type probeConstraint struct {
	indexA, indexB     int
	points             [2]probePoint
	normal             pv
	invMassA, invMassB int32
	invIA, invIB       int32
	friction           int32
	restitution        int32
	rollingMass        int32
	rollingImpulse     int32
	soft               probeSoftness
	pointCount         int
}

// probePyramid is the mirror world: the boxes, the ground, the contacts
// kept in (indexA, indexB) order, and the scalars of the step.
type probePyramid struct {
	f           *probeFormat
	sims        []probeSim
	states      []probeState
	shapes      []probeShape
	groundShape probeShape
	groundXf    Transform
	groundPoly  Polygon
	boxPoly     Polygon
	contacts    []probeContact
	spare       []probeContact
	constraints []probeConstraint
	dummy       probeState

	// lanes selects the Q32 state with working-format lanes. wideOn runs
	// the contact stages of the lanes row through the batch functions.
	lanes  bool
	wideOn bool
	wide   probeWide

	// The Q32 scalars of the lanes row, as the library computes them.
	dt32, invDt32, h32                  Q
	gravity32                           Vec2
	maxLinearSpeed32, maxAngularSpeed32 Q

	tau, dt, invDt, h, invH int32
	gravity                 pv
	maxLinearSpeed          int32
	maxAngularSpeed         int32
	pushout                 int32
	threshold               int32
	contactSoftness         probeSoftness
	staticSoftness          probeSoftness

	// maxSpeed is the largest |v| of the last finalize, in Q32.
	maxSpeed Q
}

// newProbePyramid builds the same scene as buildPyramid: unit boxes of
// density one on a static ground, sleep off.
func newProbePyramid(fracBits uint, rows int) *probePyramid {
	f := newProbeFormat(fracBits)
	centers := pyramidCenters(rows)
	half := fixed.Q32Half()
	p := &probePyramid{
		f:          f,
		sims:       make([]probeSim, len(centers)),
		states:     make([]probeState, len(centers)),
		shapes:     make([]probeShape, len(centers)),
		groundXf:   Transform{P: Vec2{Y: half.Neg()}, Q: fixed.RotIdentity()},
		groundPoly: MakeBox(fixed.Q32FromInt(rows), half),
		boxPoly:    MakeSquare(half),
		dummy:      probeState{dqc: f.one, dq32: fixed.RotIdentity()},
	}

	def := DefaultWorldDef()
	p.dt32 = fixed.Q32One().Div(fixed.Q32FromInt(60))
	p.invDt32 = fixed.Q32One().Div(p.dt32)
	p.h32 = p.dt32.Div(fixed.Q32FromInt(4))
	p.gravity32 = def.Gravity
	p.maxLinearSpeed32 = def.MaximumLinearSpeed
	p.maxAngularSpeed32 = maxRotation.Mul(p.invDt32)
	p.tau = f.fromQ(tau)
	p.gravity = f.fromVec(def.Gravity)
	p.maxLinearSpeed = f.fromQ(def.MaximumLinearSpeed)
	p.pushout = f.fromQ(def.MaxContactPushSpeed)
	p.threshold = f.fromQ(def.RestitutionThreshold)

	p.dt = f.div(f.one, 60*f.one)
	p.invDt = f.div(f.one, p.dt)
	p.h = f.div(p.dt, 4*f.one)
	p.invH = f.mul(4*f.one, p.invDt)
	p.maxAngularSpeed = f.mul(f.fromQ(maxRotation), p.invDt)

	contactHertz := f.fromQ(def.ContactHertz)
	if limit := f.mul(f.div(f.one, 8*f.one), p.invH); limit < contactHertz {
		contactHertz = limit
	}
	zeta := f.fromQ(def.ContactDampingRatio)
	p.contactSoftness = f.makeSoft(contactHertz, zeta, p.h, p.tau)
	p.staticSoftness = f.makeSoft(f.add(contactHertz, contactHertz), zeta, p.h, p.tau)

	groundAABB := ComputePolygonAABB(&p.groundPoly, p.groundXf)
	p.groundShape = probeShape{aabb: speculate(groundAABB), fatAABB: fatten(speculate(groundAABB))}

	massData := ComputePolygonMass(&p.boxPoly, fixed.Q32One())
	extent := computeShapeExtent(&shape{shapeType: PolygonShape, polygon: p.boxPoly}, massData.Center)
	for i, c := range centers {
		sim := &p.sims[i]
		sim.transform = Transform{
			P: Vec2{X: half.Mul(fixed.Q32FromInt(c[0])), Y: half.Mul(fixed.Q32FromInt(c[1]))},
			Q: fixed.RotIdentity(),
		}
		sim.localCenter = massData.Center
		sim.center = sim.transform.P.Add(sim.localCenter)
		sim.maxExtent = extent.maxExtent
		sim.invMass = f.fromQ(fixed.Q32One().Div(massData.Mass))
		sim.invInertia = f.fromQ(fixed.Q32One().Div(massData.RotationalInertia))
		p.states[i].dqc = f.one
		p.states[i].dq32 = fixed.RotIdentity()

		aabb := speculate(ComputePolygonAABB(&p.boxPoly, sim.transform))
		p.shapes[i] = probeShape{aabb: aabb, fatAABB: fatten(aabb)}
	}
	return p
}

func speculate(a AABB) AABB {
	return AABB{
		LowerBound: Vec2{X: a.LowerBound.X.Sub(speculativeDistance), Y: a.LowerBound.Y.Sub(speculativeDistance)},
		UpperBound: Vec2{X: a.UpperBound.X.Add(speculativeDistance), Y: a.UpperBound.Y.Add(speculativeDistance)},
	}
}

func fatten(a AABB) AABB {
	return AABB{
		LowerBound: Vec2{X: a.LowerBound.X.Sub(aabbMargin), Y: a.LowerBound.Y.Sub(aabbMargin)},
		UpperBound: Vec2{X: a.UpperBound.X.Add(aabbMargin), Y: a.UpperBound.Y.Add(aabbMargin)},
	}
}

func (p *probePyramid) poseOf(index int) (Transform, Vec2) {
	if index < 0 {
		return p.groundXf, Vec2Zero()
	}
	sim := &p.sims[index]
	return sim.transform, RotateVector(sim.transform.Q, sim.localCenter)
}

func (p *probePyramid) stateOf(index int) *probeState {
	if index < 0 {
		return &p.dummy
	}
	return &p.states[index]
}

// gather reads the velocity of a state into the working format, with the
// angular velocity in radians. The lanes row rounds the Q32 state to the
// nearest grid value.
func (p *probePyramid) gather(state *probeState) (pv, int32) {
	f := p.f
	if p.lanes {
		return f.fromVecNearest(state.v32), f.mul(f.fromQNearest(state.w32), p.tau)
	}
	return state.v, f.mul(state.w, p.tau)
}

// scatter writes the velocity back. The lanes row adds only the delta
// since gather to the Q32 state, so the state is never re-quantized.
func (p *probePyramid) scatter(state *probeState, v pv, w int32, v0 pv, w0 int32) {
	f := p.f
	if p.lanes {
		state.v32 = state.v32.Add(f.toVec(f.vsub(v, v0)))
		state.w32 = state.w32.Add(f.toQ(f.div(f.sub(w, w0), p.tau)))
		return
	}
	state.v = v
	state.w = f.div(w, p.tau)
}

// deltaPose reads the position deltas of the sub-step for one pair.
func (p *probePyramid) deltaPose(stateA, stateB *probeState) (dp pv, dqAc, dqAs, dqBc, dqBs int32) {
	f := p.f
	if p.lanes {
		dp = f.fromVecNearest(stateB.dp32.Sub(stateA.dp32))
		return dp, f.fromQNearest(stateA.dq32.Cos), f.fromQNearest(stateA.dq32.Sin), f.fromQNearest(stateB.dq32.Cos), f.fromQNearest(stateB.dq32.Sin)
	}
	dp = f.vsub(stateB.dp, stateA.dp)
	return dp, stateA.dqc, stateA.dqs, stateB.dqc, stateB.dqs
}

// The impulse sums of the lanes row live in Q32. accumulate adds a lane
// delta to a Q32 sum under a lower bound and returns the applied delta
// in the working format; accumulateClamped does the same under a
// symmetric bound.
func (p *probePyramid) accumulate(sum *Q, delta int32) int32 {
	f := p.f
	next := sum.Add(f.toQ(delta)).Max(fixed.Q32Zero())
	applied := f.fromQNearest(next.Sub(*sum))
	*sum = next
	return applied
}

func (p *probePyramid) accumulateClamped(sum *Q, delta int32, bound int32) int32 {
	f := p.f
	b := f.toQ(bound)
	next := sum.Add(f.toQ(delta)).Clamp(b.Neg(), b)
	applied := f.fromQNearest(next.Sub(*sum))
	*sum = next
	return applied
}

// updatePairs plays the broadphase: every pair whose fat bounds overlap
// owns one contact, in (indexA, indexB) order, and a contact survives
// while its pair overlaps. The merge walks the old list in the same order
// and allocates nothing in the steady state.
func (p *probePyramid) updatePairs() {
	next := p.spare[:0]
	old := p.contacts
	k := 0
	keep := func(a, b int) {
		for k < len(old) && (old[k].indexA < a || (old[k].indexA == a && old[k].indexB < b)) {
			k++
		}
		if k < len(old) && old[k].indexA == a && old[k].indexB == b {
			next = append(next, old[k])
			k++
			return
		}
		next = append(next, probeContact{indexA: a, indexB: b})
	}
	for i := range p.shapes {
		if AABBOverlaps(p.groundShape.fatAABB, p.shapes[i].fatAABB) {
			keep(-1, i)
		}
	}
	for i := range p.shapes {
		for j := i + 1; j < len(p.shapes); j++ {
			if AABBOverlaps(p.shapes[i].fatAABB, p.shapes[j].fatAABB) {
				keep(i, j)
			}
		}
	}
	p.spare = old
	p.contacts = next
	if cap(p.constraints) < len(next) {
		p.constraints = make([]probeConstraint, len(next))
	}
	p.constraints = p.constraints[:len(next)]
}

// updateContact mirrors updateContact: the Q32 manifold, the material mix,
// the anchor shift, and the warm-start match by id in the working format.
func (p *probePyramid) updateContact(c *probeContact) {
	f := p.f
	old := c.manifold
	oldNimp, oldTimp := c.nimp, c.timp
	oldNimp32, oldTimp32 := c.nimp32, c.timp32

	xfA, offsetA := p.poseOf(c.indexA)
	xfB, offsetB := p.poseOf(c.indexB)
	polyA := &p.boxPoly
	if c.indexA < 0 {
		polyA = &p.groundPoly
	}
	c.manifold = CollidePolygons(polyA, xfA, &p.boxPoly, xfB)

	friction := DefaultSurfaceMaterial().Friction
	c.friction = f.fromQ(defaultFrictionCallback(friction, 0, friction, 0))
	c.restitution = 0

	count := c.manifold.PointCount
	c.touching = count > 0
	for i := range count {
		mp := &c.manifold.Points[i]
		mp.AnchorA = mp.AnchorA.Sub(offsetA)
		mp.AnchorB = mp.AnchorB.Sub(offsetB)
		c.nimp[i], c.timp[i], c.tnimp[i], c.nvel[i] = 0, 0, 0, 0
		c.nimp32[i], c.timp32[i], c.tnimp32[i] = fixed.Q32Zero(), fixed.Q32Zero(), fixed.Q32Zero()
		mp.Persisted = false
		for j := range old.PointCount {
			if old.Points[j].Id == mp.Id {
				c.nimp[i] = oldNimp[j]
				c.timp[i] = oldTimp[j]
				c.nimp32[i] = oldNimp32[j]
				c.timp32[i] = oldTimp32[j]
				mp.Persisted = true
				oldNimp[j], oldTimp[j] = 0, 0
				oldNimp32[j], oldTimp32[j] = fixed.Q32Zero(), fixed.Q32Zero()
				break
			}
		}
	}
}

func (p *probePyramid) collide() {
	for i := range p.contacts {
		p.updateContact(&p.contacts[i])
	}
}

// prepare mirrors prepareContacts. The effective masses sum in the
// accumulator and store one reciprocal.
func (p *probePyramid) prepare() int {
	f := p.f
	n := 0
	for i := range p.contacts {
		cs := &p.contacts[i]
		if !cs.touching {
			continue
		}
		manifold := &cs.manifold
		pointCount := manifold.PointCount

		constraint := &p.constraints[n]
		n++
		constraint.indexA = cs.indexA
		constraint.indexB = cs.indexB
		constraint.normal = f.fromVec(manifold.Normal)
		constraint.friction = cs.friction
		constraint.restitution = cs.restitution
		constraint.rollingImpulse = cs.rolling
		constraint.pointCount = pointCount

		var vA, vB pv
		var wA, wB int32
		var mA, iA, mB, iB int32
		if cs.indexA >= 0 {
			vA, wA = p.gather(&p.states[cs.indexA])
			mA, iA = p.sims[cs.indexA].invMass, p.sims[cs.indexA].invInertia
		}
		if cs.indexB >= 0 {
			vB, wB = p.gather(&p.states[cs.indexB])
			mB, iB = p.sims[cs.indexB].invMass, p.sims[cs.indexB].invInertia
		}

		if cs.indexA < 0 || cs.indexB < 0 {
			constraint.soft = p.staticSoftness
		} else {
			constraint.soft = p.contactSoftness
		}
		constraint.invMassA, constraint.invIA = mA, iA
		constraint.invMassB, constraint.invIB = mB, iB

		k := f.widen(iA) + f.widen(iB)
		constraint.rollingMass = 0
		if k > 0 {
			constraint.rollingMass = f.narrow(f.divAcc(f.widen(f.one), k))
		}

		normal := constraint.normal
		tangent := pv{normal.y, f.neg(normal.x)}

		for j := range pointCount {
			mp := &manifold.Points[j]
			cp := &constraint.points[j]

			cp.normalImpulse = cs.nimp[j]
			cp.tangentImpulse = cs.timp[j]
			cp.totalNormalImpulse = 0
			cp.ni32, cp.ti32, cp.tni32 = cs.nimp32[j], cs.timp32[j], fixed.Q32Zero()

			rA := f.fromVec(mp.AnchorA)
			rB := f.fromVec(mp.AnchorB)
			cp.rA, cp.rB = rA, rB
			cp.baseSeparation = f.sub(f.fromQ(mp.Separation), f.dot(f.vsub(rB, rA), normal))

			rnA := f.cross(rA, normal)
			rnB := f.cross(rB, normal)
			kNormal := f.widen(mA) + f.widen(mB)
			kNormal = f.mulAdd(kNormal, f.mul(iA, rnA), rnA)
			kNormal = f.mulAdd(kNormal, f.mul(iB, rnB), rnB)
			cp.normalMass = 0
			if kNormal > 0 {
				cp.normalMass = f.narrow(f.divAcc(f.widen(f.one), kNormal))
			}

			rtA := f.cross(rA, tangent)
			rtB := f.cross(rB, tangent)
			kTangent := f.widen(mA) + f.widen(mB)
			kTangent = f.mulAdd(kTangent, f.mul(iA, rtA), rtA)
			kTangent = f.mulAdd(kTangent, f.mul(iB, rtB), rtB)
			cp.tangentMass = 0
			if kTangent > 0 {
				cp.tangentMass = f.narrow(f.divAcc(f.widen(f.one), kTangent))
			}

			vrA := f.vadd(vA, f.crossSV(wA, rA))
			vrB := f.vadd(vB, f.crossSV(wB, rB))
			cp.relativeVelocity = f.dot(f.vsub(vrB, vrA), normal)
		}
	}
	return n
}

// warmStart mirrors warmStartContacts.
func (p *probePyramid) warmStart(count int) {
	f := p.f
	for i := range count {
		constraint := &p.constraints[i]
		stateA := p.stateOf(constraint.indexA)
		stateB := p.stateOf(constraint.indexB)
		vA, wA := p.gather(stateA)
		vB, wB := p.gather(stateB)
		vA0, wA0, vB0, wB0 := vA, wA, vB, wB

		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB

		normal := constraint.normal
		tangent := pv{normal.y, f.neg(normal.x)}

		for j := range constraint.pointCount {
			cp := &constraint.points[j]
			ni, ti := cp.normalImpulse, cp.tangentImpulse
			if p.lanes {
				ni, ti = f.fromQNearest(cp.ni32), f.fromQNearest(cp.ti32)
			}
			P := f.vadd(f.vscale(normal, ni), f.vscale(tangent, ti))
			wA = f.sub(wA, f.mul(iA, f.cross(cp.rA, P)))
			vA = f.vmulAdd(vA, f.neg(mA), P)
			wB = f.add(wB, f.mul(iB, f.cross(cp.rB, P)))
			vB = f.vmulAdd(vB, mB, P)
		}

		wA = f.sub(wA, f.mul(iA, constraint.rollingImpulse))
		wB = f.add(wB, f.mul(iB, constraint.rollingImpulse))

		p.scatter(stateA, vA, wA, vA0, wA0)
		p.scatter(stateB, vB, wB, vB0, wB0)
	}
}

// solve mirrors solveContacts, with and without the bias.
func (p *probePyramid) solve(count int, useBias bool) {
	f := p.f
	for i := range count {
		constraint := &p.constraints[i]
		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB

		stateA := p.stateOf(constraint.indexA)
		stateB := p.stateOf(constraint.indexB)
		vA, wA := p.gather(stateA)
		vB, wB := p.gather(stateB)
		vA0, wA0, vB0, wB0 := vA, wA, vB, wB
		dp, dqAc, dqAs, dqBc, dqBs := p.deltaPose(stateA, stateB)

		normal := constraint.normal
		tangent := pv{normal.y, f.neg(normal.x)}
		friction := constraint.friction
		soft := constraint.soft

		pointCount := constraint.pointCount
		var totalNormalImpulse int32

		for j := range pointCount {
			cp := &constraint.points[j]
			rA, rB := cp.rA, cp.rB

			ds := f.vadd(dp, f.vsub(f.rotate(dqBc, dqBs, rB), f.rotate(dqAc, dqAs, rA)))
			s := f.add(cp.baseSeparation, f.dot(ds, normal))

			var velocityBias int32
			massScale := f.one
			var impulseScale int32
			if s > 0 {
				velocityBias = f.mul(s, p.invH)
			} else if useBias {
				velocityBias = f.max(f.mul(soft.biasRate, s), f.neg(p.pushout))
				massScale = soft.massScale
				impulseScale = soft.impulseScale
			}

			vrA := f.vadd(vA, f.crossSV(wA, rA))
			vrB := f.vadd(vB, f.crossSV(wB, rB))
			vn := f.dot(f.vsub(vrB, vrA), normal)

			ni := cp.normalImpulse
			if p.lanes {
				ni = f.fromQNearest(cp.ni32)
			}
			impulse := f.sub(f.mul(f.mul(f.neg(cp.normalMass), massScale), f.add(vn, velocityBias)), f.mul(impulseScale, ni))

			if p.lanes {
				impulse = p.accumulate(&cp.ni32, impulse)
				cp.tni32 = cp.tni32.Add(cp.ni32)
				totalNormalImpulse = f.add(totalNormalImpulse, f.fromQNearest(cp.ni32))
			} else {
				newImpulse := f.max(f.add(cp.normalImpulse, impulse), 0)
				impulse = f.sub(newImpulse, cp.normalImpulse)
				cp.normalImpulse = newImpulse
				cp.totalNormalImpulse = f.add(cp.totalNormalImpulse, newImpulse)
				totalNormalImpulse = f.add(totalNormalImpulse, newImpulse)
			}

			P := f.vscale(normal, impulse)
			vA = f.vmulSub(vA, mA, P)
			wA = f.sub(wA, f.mul(iA, f.cross(rA, P)))
			vB = f.vmulAdd(vB, mB, P)
			wB = f.add(wB, f.mul(iB, f.cross(rB, P)))
		}

		for j := range pointCount {
			cp := &constraint.points[j]
			rA, rB := cp.rA, cp.rB

			vrB := f.vadd(vB, f.crossSV(wB, rB))
			vrA := f.vadd(vA, f.crossSV(wA, rA))
			vt := f.dot(f.vsub(vrB, vrA), tangent)

			impulse := f.mul(cp.tangentMass, f.neg(vt))

			if p.lanes {
				maxFriction := f.mul(friction, f.fromQNearest(cp.ni32))
				impulse = p.accumulateClamped(&cp.ti32, impulse, maxFriction)
			} else {
				maxFriction := f.mul(friction, cp.normalImpulse)
				newImpulse := f.clamp(f.add(cp.tangentImpulse, impulse), f.neg(maxFriction), maxFriction)
				impulse = f.sub(newImpulse, cp.tangentImpulse)
				cp.tangentImpulse = newImpulse
			}

			P := f.vscale(tangent, impulse)
			vA = f.vmulSub(vA, mA, P)
			wA = f.sub(wA, f.mul(iA, f.cross(rA, P)))
			vB = f.vmulAdd(vB, mB, P)
			wB = f.add(wB, f.mul(iB, f.cross(rB, P)))
		}

		{
			deltaLambda := f.mul(f.neg(constraint.rollingMass), f.sub(wB, wA))
			lambda := constraint.rollingImpulse
			// The boxes have no rolling resistance.
			maxLambda := f.mul(0, totalNormalImpulse)
			constraint.rollingImpulse = f.clamp(f.add(lambda, deltaLambda), f.neg(maxLambda), maxLambda)
			deltaLambda = f.sub(constraint.rollingImpulse, lambda)
			wA = f.sub(wA, f.mul(iA, deltaLambda))
			wB = f.add(wB, f.mul(iB, deltaLambda))
		}

		p.scatter(stateA, vA, wA, vA0, wA0)
		p.scatter(stateB, vB, wB, vB0, wB0)
	}
}

// restitution mirrors applyRestitution. The boxes have none, so the stage
// returns at the first test as the Q32 stage does.
func (p *probePyramid) restitution(count int) {
	f := p.f
	for i := range count {
		constraint := &p.constraints[i]
		if constraint.restitution == 0 {
			continue
		}
		mA, iA := constraint.invMassA, constraint.invIA
		mB, iB := constraint.invMassB, constraint.invIB

		stateA := p.stateOf(constraint.indexA)
		stateB := p.stateOf(constraint.indexB)
		vA, wA := p.gather(stateA)
		vB, wB := p.gather(stateB)
		vA0, wA0, vB0, wB0 := vA, wA, vB, wB
		normal := constraint.normal

		for j := range constraint.pointCount {
			cp := &constraint.points[j]
			total := cp.totalNormalImpulse != 0
			if p.lanes {
				total = !cp.tni32.Eq(fixed.Q32Zero())
			}
			if f.neg(p.threshold) < cp.relativeVelocity || !total {
				continue
			}
			rA, rB := cp.rA, cp.rB
			vrB := f.vadd(vB, f.crossSV(wB, rB))
			vrA := f.vadd(vA, f.crossSV(wA, rA))
			vn := f.dot(f.vsub(vrB, vrA), normal)

			impulse := f.mul(f.neg(cp.normalMass), f.add(vn, f.mul(constraint.restitution, cp.relativeVelocity)))
			if p.lanes {
				impulse = p.accumulate(&cp.ni32, impulse)
				cp.tni32 = cp.tni32.Add(f.toQ(impulse))
			} else {
				newImpulse := f.max(f.add(cp.normalImpulse, impulse), 0)
				impulse = f.sub(newImpulse, cp.normalImpulse)
				cp.normalImpulse = newImpulse
				cp.totalNormalImpulse = f.add(cp.totalNormalImpulse, impulse)
			}

			P := f.vscale(normal, impulse)
			vA = f.vmulSub(vA, mA, P)
			wA = f.sub(wA, f.mul(iA, f.cross(rA, P)))
			vB = f.vmulAdd(vB, mB, P)
			wB = f.add(wB, f.mul(iB, f.cross(rB, P)))
		}

		p.scatter(stateA, vA, wA, vA0, wA0)
		p.scatter(stateB, vB, wB, vB0, wB0)
	}
}

// store mirrors storeImpulses.
func (p *probePyramid) store() {
	n := 0
	for i := range p.contacts {
		cs := &p.contacts[i]
		if !cs.touching {
			continue
		}
		constraint := &p.constraints[n]
		n++
		for j := range cs.manifold.PointCount {
			cs.nimp[j] = constraint.points[j].normalImpulse
			cs.timp[j] = constraint.points[j].tangentImpulse
			cs.tnimp[j] = constraint.points[j].totalNormalImpulse
			cs.nvel[j] = constraint.points[j].relativeVelocity
			cs.nimp32[j] = constraint.points[j].ni32
			cs.timp32[j] = constraint.points[j].ti32
			cs.tnimp32[j] = constraint.points[j].tni32
		}
		cs.rolling = constraint.rollingImpulse
	}
}

// integrateVelocities32 follows integrateVelocitiesTask in Q32 for the
// lanes row: no damping, no force, no torque.
func (p *probePyramid) integrateVelocities32() {
	zero := fixed.Q32Zero()
	one := fixed.Q32One()
	h := p.h32
	maxLinearSpeedSquared := p.maxLinearSpeed32.Mul(p.maxLinearSpeed32)
	maxAngularSpeedSquared := p.maxAngularSpeed32.Mul(p.maxAngularSpeed32)
	for i := range p.sims {
		sim := &p.sims[i]
		state := &p.states[i]
		v := state.v32
		omega := state.w32

		linearDamping := one.Add(h.Mul(zero))
		angularDamping := one.Add(h.Mul(zero))
		gravityScale := zero
		if sim.invMass > 0 {
			gravityScale = one
		}
		linearVelocityDelta := Vec2Zero().Mul(h.Mul(p.f.toQ(sim.invMass))).Add(p.gravity32.Mul(h.Mul(gravityScale)))
		angularVelocityDelta := h.Mul(p.f.toQ(sim.invInertia)).Mul(zero).Div(tau)

		v = Vec2{X: linearVelocityDelta.X.Add(v.X.Div(linearDamping)), Y: linearVelocityDelta.Y.Add(v.Y.Div(linearDamping))}
		omega = angularVelocityDelta.Add(omega.Div(angularDamping))

		if maxLinearSpeedSquared.Less(v.Dot(v)) {
			v = v.Mul(p.maxLinearSpeed32.Div(v.Len()))
			sim.isSpeedCapped = true
		}
		if maxAngularSpeedSquared.Less(omega.Mul(omega)) {
			omega = omega.Mul(p.maxAngularSpeed32.Div(omega.Abs()))
			sim.isSpeedCapped = true
		}
		state.v32 = v
		state.w32 = omega
	}
}

// integratePositions32 follows integratePositionsTask in Q32.
func (p *probePyramid) integratePositions32() {
	for i := range p.states {
		state := &p.states[i]
		state.dq32 = IntegrateRotation(state.dq32, p.h32.Mul(state.w32))
		state.dp32 = MulAdd(state.dp32, p.h32, state.v32)
	}
}

// integrateVelocities mirrors integrateVelocitiesTask. The speed limit
// squared does not fit the working format, so the linear test compares
// two accumulators.
func (p *probePyramid) integrateVelocities() {
	if p.lanes {
		p.integrateVelocities32()
		return
	}
	f := p.f
	maxLinearSpeedSquared := f.mulAdd(0, p.maxLinearSpeed, p.maxLinearSpeed)
	maxAngularSpeedSquared := f.mul(p.maxAngularSpeed, p.maxAngularSpeed)

	for i := range p.sims {
		sim := &p.sims[i]
		state := &p.states[i]
		v := state.v
		omega := state.w

		// The boxes have no damping, no force and no torque; the lines stay
		// so the arithmetic mirrors the stage.
		linearDamping := f.add(f.one, f.mul(p.h, 0))
		angularDamping := f.add(f.one, f.mul(p.h, 0))

		var gravityScale int32
		if sim.invMass > 0 {
			gravityScale = f.one
		}

		linearVelocityDelta := f.vadd(f.vscale(pv{}, f.mul(p.h, sim.invMass)), f.vscale(p.gravity, f.mul(p.h, gravityScale)))
		angularVelocityDelta := f.div(f.mul(f.mul(p.h, sim.invInertia), 0), p.tau)

		v = pv{f.add(linearVelocityDelta.x, f.div(v.x, linearDamping)), f.add(linearVelocityDelta.y, f.div(v.y, linearDamping))}
		omega = f.add(angularVelocityDelta, f.div(omega, angularDamping))

		if vv := f.mulAdd(f.mulAdd(0, v.x, v.x), v.y, v.y); maxLinearSpeedSquared < vv {
			ratio := f.div(p.maxLinearSpeed, f.narrow(f.sqrtAcc(vv)))
			v = f.vscale(v, ratio)
			sim.isSpeedCapped = true
		}
		if maxAngularSpeedSquared < f.mul(omega, omega) {
			ratio := f.div(p.maxAngularSpeed, f.abs(omega))
			omega = f.mul(omega, ratio)
			sim.isSpeedCapped = true
		}

		state.v = v
		state.w = omega
	}
}

// integratePositions mirrors integratePositionsTask. The rotation
// normalizes by the reference division, not by the Q32 unit pair.
func (p *probePyramid) integratePositions() {
	if p.lanes {
		p.integratePositions32()
		return
	}
	f := p.f
	for i := range p.states {
		state := &p.states[i]

		d := f.mul(f.mul(p.h, state.w), p.tau)
		c := f.sub(state.dqc, f.mul(d, state.dqs))
		s := f.add(state.dqs, f.mul(d, state.dqc))
		mag := f.narrow(f.sqrtAcc(f.mulAdd(f.mulAdd(0, c, c), s, s)))
		if mag > 0 {
			c, s = f.div(c, mag), f.div(s, mag)
		} else {
			c, s = f.one, 0
		}
		state.dqc, state.dqs = c, s

		state.dp = f.vmulAdd(state.dp, p.h, state.v)
	}
}

// finalize mirrors finalizeBodiesTask on the Q32 side: the deltas cross
// the boundary into the pose, and the bounds refresh.
func (p *probePyramid) finalize() {
	f := p.f
	p.maxSpeed = fixed.Q32Zero()
	for i := range p.sims {
		sim := &p.sims[i]
		state := &p.states[i]

		if p.lanes {
			sim.center = sim.center.Add(state.dp32)
			sim.transform.Q = NormalizeRot(MulRot(state.dq32, sim.transform.Q))
			p.maxSpeed = p.maxSpeed.Max(state.v32.Len())
			state.dp32 = Vec2Zero()
			state.dq32 = fixed.RotIdentity()
		} else {
			sim.center = sim.center.Add(f.toVec(state.dp))
			dq := Rot{Cos: f.toQ(state.dqc), Sin: f.toQ(state.dqs)}
			sim.transform.Q = NormalizeRot(MulRot(dq, sim.transform.Q))

			speed := f.toQ(f.narrow(f.sqrtAcc(f.mulAdd(f.mulAdd(0, state.v.x, state.v.x), state.v.y, state.v.y))))
			p.maxSpeed = p.maxSpeed.Max(speed)

			state.dp = pv{}
			state.dqc, state.dqs = f.one, 0
		}

		sim.transform.P = sim.center.Sub(RotateVector(sim.transform.Q, sim.localCenter))
		sim.isSpeedCapped = false

		shape := &p.shapes[i]
		shape.aabb = speculate(ComputePolygonAABB(&p.boxPoly, sim.transform))
		if !AABBContains(shape.fatAABB, shape.aabb) {
			shape.fatAABB = fatten(shape.aabb)
		}
	}
}

// step mirrors Step over the pyramid.
func (p *probePyramid) step() {
	p.updatePairs()
	p.collide()
	count := p.prepare()
	if p.wideOn {
		p.wideBuild(count)
	}
	for range 4 {
		p.integrateVelocities()
		if p.wideOn {
			p.wideWarmStart()
			p.wideSolve(true)
			p.integratePositions()
			p.wideSolve(false)
			continue
		}
		p.warmStart(count)
		p.solve(count, true)
		p.integratePositions()
		p.solve(count, false)
	}
	if p.wideOn {
		p.wideStore(count)
	}
	p.restitution(count)
	p.store()
	p.finalize()
}

// checksum folds the pose, the state and the impulses of the mirror.
func (p *probePyramid) checksum() uint64 {
	h := fnvOffsetBasis
	for i := range p.sims {
		sim := &p.sims[i]
		state := &p.states[i]
		h = checksumVec2(h, sim.center)
		h = checksumRot(h, sim.transform.Q)
		if p.lanes {
			h = checksumVec2(h, state.v32)
			h = fnvFold(h, uint64(state.w32.Raw()))
			continue
		}
		h = fnvFold(h, uint64(uint32(state.v.x)))
		h = fnvFold(h, uint64(uint32(state.v.y)))
		h = fnvFold(h, uint64(uint32(state.w)))
	}
	for i := range p.contacts {
		c := &p.contacts[i]
		h = fnvFold(h, uint64(c.manifold.PointCount))
		for j := range c.manifold.PointCount {
			if p.lanes {
				h = fnvFold(h, uint64(c.nimp32[j].Raw()))
				h = fnvFold(h, uint64(c.timp32[j].Raw()))
				continue
			}
			h = fnvFold(h, uint64(uint32(c.nimp[j])))
			h = fnvFold(h, uint64(uint32(c.timp[j])))
		}
	}
	return h
}

// topY returns the center height of the last box, which is the top of the
// pyramid, as a float for the report.
func (p *probePyramid) topY() float64 {
	return qFloat(p.sims[len(p.sims)-1].center.Y)
}

// qFloat converts a Q32 for the report only.
func qFloat(q Q) float64 { return float64(q.Raw()) / (1 << 32) }

const (
	probeSteps      = 1000
	probeTailSteps  = 100
	probeRestLimit  = 0.02
	probeSleepLimit = 0.05
)

// probeRun holds the measures of one scene run.
type probeRun struct {
	topY           float64
	maxSpeed       float64
	checksum       uint64
	saturations    int
	accSaturations int
	fixedEvents    uint64
}

// runProbe runs the mirror over the scene and collects the criteria.
func runProbe(fracBits uint, nearest, lanes, wide bool) probeRun {
	fixed.ResetSaturationCount()
	p := newProbePyramid(fracBits, pyramidRows)
	p.f.nearest = nearest
	p.lanes = lanes
	p.wideOn = wide
	var run probeRun
	for i := range probeSteps {
		p.step()
		if i >= probeSteps-probeTailSteps {
			run.maxSpeed = math.Max(run.maxSpeed, qFloat(p.maxSpeed))
		}
	}
	run.topY = p.topY()
	run.checksum = p.checksum()
	run.saturations = p.f.saturations
	run.accSaturations = p.f.accSaturations
	run.fixedEvents = fixed.SaturationCount()
	return run
}

// runReference runs the Q32 engine over the same scene through Step and
// the broadphase.
func runReference() probeRun {
	fixed.ResetSaturationCount()
	def := DefaultWorldDef()
	def.EnableSleep = false
	worldId := CreateWorld(&def)
	defer DestroyWorld(worldId)
	top := buildPyramid(worldId, pyramidRows)
	w := getWorldFromId(worldId)

	dt := fixed.Q32One().Div(fixed.Q32FromInt(60))
	var run probeRun
	for i := range probeSteps {
		worldId.Step(dt, 4)
		if i >= probeSteps-probeTailSteps {
			for _, state := range w.solverSets[awakeSet].bodyStates {
				run.maxSpeed = math.Max(run.maxSpeed, qFloat(state.linearVelocity.Len()))
			}
		}
	}
	b := &w.bodies[top.index1-1]
	run.topY = qFloat(w.solverSets[b.setIndex].bodySims[b.localIndex].center.Y)
	run.checksum = Checksum(worldId)
	run.fixedEvents = fixed.SaturationCount()
	return run
}

// TestProbeFormatMatchesQ16AndQ48 proves that the working format with 16
// fraction bits is fixed.Q16 and its accumulator is fixed.Q48, operation
// by operation, on random operands.
func TestProbeFormatMatchesQ16AndQ48(t *testing.T) {
	f := newProbeFormat(16)
	rng := rand.New(rand.NewSource(1))
	for range 100000 {
		a := int32(rng.Uint32())
		b := int32(rng.Uint32())
		qa, qb := fixed.Q16FromRaw(a), fixed.Q16FromRaw(b)

		if got, want := f.mul(a, b), qa.Mul(qb).Raw(); got != want {
			t.Fatalf("mul(%d, %d) = %d, Q16 gives %d", a, b, got, want)
		}
		if b != 0 {
			if got, want := f.div(a, b), qa.Div(qb).Raw(); got != want {
				t.Fatalf("div(%d, %d) = %d, Q16 gives %d", a, b, got, want)
			}
		}
		if a >= 0 {
			if got, want := f.sqrt(a), qa.Sqrt().Raw(); got != want {
				t.Fatalf("sqrt(%d) = %d, Q16 gives %d", a, got, want)
			}
		}

		acc := rng.Int63n(1<<40) - 1<<39
		q := fixed.Q48FromRaw(acc)
		if got, want := f.mulAdd(acc, a, b), q.MulAdd16(qa, qb).Raw(); got != want {
			t.Fatalf("mulAdd(%d, %d, %d) = %d, Q48 gives %d", acc, a, b, got, want)
		}
		if got, want := f.mulSub(acc, a, b), q.Sub(fixed.Q48FromRaw((int64(a)*int64(b))>>16)).Raw(); got != want {
			t.Fatalf("mulSub(%d, %d, %d) = %d, Q48 gives %d", acc, a, b, got, want)
		}
		if acc > 0 {
			if got, want := f.divAcc(f.widen(f.one), acc), fixed.Q48One().Div(q).Raw(); got != want {
				t.Fatalf("divAcc(one, %d) = %d, Q48 gives %d", acc, got, want)
			}
			if got, want := f.sqrtAcc(acc), q.Sqrt().Raw(); got != want {
				t.Fatalf("sqrtAcc(%d) = %d, Q48 gives %d", acc, got, want)
			}
		}
		if got, want := f.narrow(acc), q.ToQ16().Raw(); got != want {
			t.Fatalf("narrow(%d) = %d, Q48 gives %d", acc, got, want)
		}
		if got, want := f.widen(a), qa.ToQ48().Raw(); got != want {
			t.Fatalf("widen(%d) = %d, Q16 gives %d", a, got, want)
		}

		q32 := fixed.Q32FromRaw(int64(rng.Uint64()))
		if got, want := f.fromQ(q32), q32.ToQ16().Raw(); got != want {
			t.Fatalf("fromQ(%d) = %d, Q32 gives %d", q32.Raw(), got, want)
		}
		if got, want := f.toQ(a), qa.ToQ32(); !got.Eq(want) {
			t.Fatalf("toQ(%d) = %d, Q16 gives %d", a, got.Raw(), want.Raw())
		}
	}

	// The boundary of the accumulator: sums, differences and quotients
	// that leave int64 saturate like Q48 and count once each.
	fixed.ResetSaturationCount()
	f.accSaturations = 0
	edges := []int64{math.MaxInt64, math.MaxInt64 - 1, math.MinInt64, math.MinInt64 + 1, 1 << 62, -1 << 62, 1 << 47, -1 << 47, 1, -1, 0}
	factors := []int32{math.MaxInt32, math.MinInt32, 1 << 16, -1 << 16, 1, -1, 0}
	for _, acc := range edges {
		q := fixed.Q48FromRaw(acc)
		for _, a := range factors {
			for _, b := range factors {
				qa, qb := fixed.Q16FromRaw(a), fixed.Q16FromRaw(b)
				if got, want := f.mulAdd(acc, a, b), q.MulAdd16(qa, qb).Raw(); got != want {
					t.Fatalf("mulAdd(%d, %d, %d) = %d, Q48 gives %d", acc, a, b, got, want)
				}
				if got, want := f.mulSub(acc, a, b), q.Sub(fixed.Q48FromRaw((int64(a)*int64(b))>>16)).Raw(); got != want {
					t.Fatalf("mulSub(%d, %d, %d) = %d, Q48 gives %d", acc, a, b, got, want)
				}
			}
		}
		for _, d := range edges {
			if d == 0 {
				continue
			}
			if got, want := f.divAcc(acc, d), q.Div(fixed.Q48FromRaw(d)).Raw(); got != want {
				t.Fatalf("divAcc(%d, %d) = %d, Q48 gives %d", acc, d, got, want)
			}
		}
		if acc >= 0 {
			if got, want := f.sqrtAcc(acc), q.Sqrt().Raw(); got != want {
				t.Fatalf("sqrtAcc(%d) = %d, Q48 gives %d", acc, got, want)
			}
		}
	}
	if f.accSaturations == 0 {
		t.Fatal("the boundary cases saturated nothing")
	}
	if got, want := uint64(f.accSaturations), fixed.SaturationCount(); got != want {
		t.Fatalf("the accumulator counted %d saturations, Q48 counted %d", got, want)
	}
	fixed.ResetSaturationCount()
}

// probeRow is one row of the probe table: a format, its rounding, and
// whether the format is a lane format over a Q32 state.
type probeRow struct {
	fracBits uint
	nearest  bool
	lanes    bool
	wide     bool
	witness  uint64
}

// probeRows lists the plan rows first and the diagnosis row last. The
// witness pins the checksum of each row: a run on another architecture
// and a later run here must reproduce it.
var probeRows = []probeRow{
	{fracBits: 16, witness: 11199930024838461989},
	{fracBits: 12, witness: 6491751349712592663},
	{fracBits: 16, nearest: true, witness: 3093283145204991723},
	{fracBits: 16, nearest: true, lanes: true, witness: 17243034577844031924},
	{fracBits: 16, nearest: false, lanes: true, witness: 3884108997595085682},
	{fracBits: 16, nearest: false, lanes: true, wide: true, witness: 3624099612550400531},
}

// TestProbeSolverFormats runs the scene in Q32 and in the mirror for every
// row, and reports the criteria of the probe. Only the determinism fails
// the test: the rest and the jitter are the measurement, and the note of
// the probe records them.
func TestProbeSolverFormats(t *testing.T) {
	if testing.Short() {
		t.Skip("the probe runs the pyramid for 1000 steps")
	}

	reference := runReference()
	t.Logf("Q32: top %.5f m, max |v| %.5f m/s, checksum %d, fixed saturations %d",
		reference.topY, reference.maxSpeed, reference.checksum, reference.fixedEvents)

	verdict := func(ok bool) string {
		if ok {
			return "pass"
		}
		return "fail"
	}

	for _, row := range probeRows {
		first := runProbe(row.fracBits, row.nearest, row.lanes, row.wide)
		second := runProbe(row.fracBits, row.nearest, row.lanes, row.wide)

		rest := math.Abs(first.topY - reference.topY)
		t.Logf("fracBits %d nearest %v lanes %v wide %v (%s): top %.5f m (delta %.5f, %s), max |v| %.5f m/s (%s), checksum %d, grid saturations %d, accumulator saturations %d, fixed saturations %d (%s)",
			row.fracBits, row.nearest, row.lanes, row.wide, fixed.BatchPath(), first.topY, rest, verdict(rest <= probeRestLimit),
			first.maxSpeed, verdict(first.maxSpeed < probeSleepLimit), first.checksum,
			first.saturations, first.accSaturations, first.fixedEvents,
			verdict(first.saturations == 0 && first.accSaturations == 0 && first.fixedEvents == 0))

		if first.checksum != second.checksum {
			t.Errorf("fracBits %d nearest %v lanes %v: two runs gave checksums %d and %d", row.fracBits, row.nearest, row.lanes, first.checksum, second.checksum)
		}
		if row.witness != 0 && row.witness != first.checksum {
			t.Errorf("fracBits %d nearest %v lanes %v: checksum %d, the witness is %d", row.fracBits, row.nearest, row.lanes, first.checksum, row.witness)
		}
	}
}

// BenchmarkProbeStepPyramid16 and 12 measure the mirror step over the
// same pyramid as BenchmarkStepPyramid and BenchmarkStepPyramidF64.
func benchmarkProbeStepPyramid(b *testing.B, fracBits uint, nearest, lanes, wide bool) {
	p := newProbePyramid(fracBits, pyramidRows)
	p.f.nearest = nearest
	p.lanes = lanes
	p.wideOn = wide
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		p.step()
	}
	b.StopTimer()
	if len(p.contacts) == 0 {
		b.Fatal("the pyramid lost its contacts")
	}
}

func BenchmarkProbeStepPyramid16(b *testing.B) { benchmarkProbeStepPyramid(b, 16, false, false, false) }
func BenchmarkProbeStepPyramid12(b *testing.B) { benchmarkProbeStepPyramid(b, 12, false, false, false) }
func BenchmarkProbeStepPyramid16Nearest(b *testing.B) {
	benchmarkProbeStepPyramid(b, 16, true, false, false)
}
func BenchmarkProbeStepPyramidLanes(b *testing.B) {
	benchmarkProbeStepPyramid(b, 16, true, true, false)
}
func BenchmarkProbeStepPyramidLanesFloor(b *testing.B) {
	benchmarkProbeStepPyramid(b, 16, false, true, false)
}
func BenchmarkProbeStepPyramidWide(b *testing.B) {
	b.Logf("batch path %s", fixed.BatchPath())
	benchmarkProbeStepPyramid(b, 16, false, true, true)
}
