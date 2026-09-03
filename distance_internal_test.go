package dbox2d

import (
	"math"
	"math/rand"
	"testing"

	"github.com/dhannyell/fixed"
)

// The float64 mirror of the GJK solver follows distance.go line by line.
// It is a sanity check of the values, never an oracle of the bits, and the
// benchmark in bench_test.go reuses it.

type f64Vec struct{ x, y float64 }

func (a f64Vec) add(b f64Vec) f64Vec    { return f64Vec{a.x + b.x, a.y + b.y} }
func (a f64Vec) sub(b f64Vec) f64Vec    { return f64Vec{a.x - b.x, a.y - b.y} }
func (a f64Vec) dot(b f64Vec) float64   { return a.x*b.x + a.y*b.y }
func (a f64Vec) cross(b f64Vec) float64 { return a.x*b.y - a.y*b.x }
func (a f64Vec) neg() f64Vec            { return f64Vec{-a.x, -a.y} }

type f64Transform struct {
	px, py, qc, qs float64
}

func (t f64Transform) apply(p f64Vec) f64Vec {
	return f64Vec{t.qc*p.x - t.qs*p.y + t.px, t.qs*p.x + t.qc*p.y + t.py}
}

func (t f64Transform) rotate(v f64Vec) f64Vec {
	return f64Vec{t.qc*v.x - t.qs*v.y, t.qs*v.x + t.qc*v.y}
}

// invMul mirrors InvMulTransforms.
func (t f64Transform) invMul(b f64Transform) f64Transform {
	qc := t.qc*b.qc + t.qs*b.qs
	qs := t.qc*b.qs - t.qs*b.qc
	dx, dy := b.px-t.px, b.py-t.py
	return f64Transform{
		px: t.qc*dx + t.qs*dy,
		py: -t.qs*dx + t.qc*dy,
		qc: qc,
		qs: qs,
	}
}

type f64Proxy struct {
	points [MaxPolygonVertices]f64Vec
	count  int
	radius float64
}

type f64SimplexVertex struct {
	wA, wB, w      f64Vec
	a              float64
	indexA, indexB int
}

type f64Simplex struct {
	v1, v2, v3 f64SimplexVertex
	count      int
}

type f64DistanceOutput struct {
	pointA, pointB, normal f64Vec
	distance               float64
	iterations             int
}

func f64Weight2(a1 float64, w1 f64Vec, a2 float64, w2 f64Vec) f64Vec {
	return f64Vec{a1*w1.x + a2*w2.x, a1*w1.y + a2*w2.y}
}

func f64Weight3(a1 float64, w1 f64Vec, a2 float64, w2 f64Vec, a3 float64, w3 f64Vec) f64Vec {
	return f64Vec{a1*w1.x + a2*w2.x + a3*w3.x, a1*w1.y + a2*w2.y + a3*w3.y}
}

func f64FindSupport(proxy *f64Proxy, d f64Vec) int {
	best := 0
	bestValue := proxy.points[0].dot(d)
	for i := 1; i < proxy.count; i++ {
		value := proxy.points[i].dot(d)
		if value > bestValue {
			best = i
			bestValue = value
		}
	}
	return best
}

func f64WitnessPoints(s *f64Simplex) (a, b f64Vec) {
	switch s.count {
	case 1:
		return s.v1.wA, s.v1.wB
	case 2:
		return f64Weight2(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA), f64Weight2(s.v1.a, s.v1.wB, s.v2.a, s.v2.wB)
	default:
		a = f64Weight3(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA, s.v3.a, s.v3.wA)
		return a, a
	}
}

func f64SolveSimplex2(s *f64Simplex) f64Vec {
	w1, w2 := s.v1.w, s.v2.w
	e12 := w2.sub(w1)

	d12_2 := -w1.dot(e12)
	if d12_2 <= 0 {
		s.v1.a = 1
		s.count = 1
		return w1.neg()
	}

	d12_1 := w2.dot(e12)
	if d12_1 <= 0 {
		s.v2.a = 1
		s.count = 1
		s.v1 = s.v2
		return w2.neg()
	}

	inv := 1 / (d12_1 + d12_2)
	s.v1.a = d12_1 * inv
	s.v2.a = d12_2 * inv
	s.count = 2
	c := w1.add(w2).cross(e12)
	return f64Vec{-c * e12.y, c * e12.x}
}

func f64SolveSimplex3(s *f64Simplex) f64Vec {
	w1, w2, w3 := s.v1.w, s.v2.w, s.v3.w

	e12 := w2.sub(w1)
	d12_1 := w2.dot(e12)
	d12_2 := -w1.dot(e12)

	e13 := w3.sub(w1)
	d13_1 := w3.dot(e13)
	d13_2 := -w1.dot(e13)

	e23 := w3.sub(w2)
	d23_1 := w3.dot(e23)
	d23_2 := -w2.dot(e23)

	n123 := e12.cross(e13)
	d123_1 := n123 * w2.cross(w3)
	d123_2 := n123 * w3.cross(w1)
	d123_3 := n123 * w1.cross(w2)

	if d12_2 <= 0 && d13_2 <= 0 {
		s.v1.a = 1
		s.count = 1
		return w1.neg()
	}

	if d12_1 > 0 && d12_2 > 0 && d123_3 <= 0 {
		inv := 1 / (d12_1 + d12_2)
		s.v1.a = d12_1 * inv
		s.v2.a = d12_2 * inv
		s.count = 2
		c := w1.add(w2).cross(e12)
		return f64Vec{-c * e12.y, c * e12.x}
	}

	if d13_1 > 0 && d13_2 > 0 && d123_2 <= 0 {
		inv := 1 / (d13_1 + d13_2)
		s.v1.a = d13_1 * inv
		s.v3.a = d13_2 * inv
		s.count = 2
		s.v2 = s.v3
		c := w1.add(w3).cross(e13)
		return f64Vec{-c * e13.y, c * e13.x}
	}

	if d12_1 <= 0 && d23_2 <= 0 {
		s.v2.a = 1
		s.count = 1
		s.v1 = s.v2
		return w2.neg()
	}

	if d13_1 <= 0 && d23_1 <= 0 {
		s.v3.a = 1
		s.count = 1
		s.v1 = s.v3
		return w3.neg()
	}

	if d23_1 > 0 && d23_2 > 0 && d123_1 <= 0 {
		inv := 1 / (d23_1 + d23_2)
		s.v2.a = d23_1 * inv
		s.v3.a = d23_2 * inv
		s.count = 2
		s.v1 = s.v3
		c := w2.add(w3).cross(e23)
		return f64Vec{-c * e23.y, c * e23.x}
	}

	inv := 1 / (d123_1 + d123_2 + d123_3)
	s.v1.a = d123_1 * inv
	s.v2.a = d123_2 * inv
	s.v3.a = d123_3 * inv
	s.count = 3
	return f64Vec{}
}

type f64SimplexCache struct {
	count  int
	indexA [3]int
	indexB [3]int
}

// shapeDistanceF64 mirrors ShapeDistance, cache included.
func shapeDistanceF64(proxyA, proxyB *f64Proxy, xfA, xfB f64Transform, useRadii bool, cache *f64SimplexCache) f64DistanceOutput {
	var output f64DistanceOutput

	var localB f64Proxy
	{
		xf := xfA.invMul(xfB)
		localB.count = proxyB.count
		localB.radius = proxyB.radius
		for i := range localB.count {
			localB.points[i] = xf.apply(proxyB.points[i])
		}
	}

	var simplex f64Simplex
	vertices := [3]*f64SimplexVertex{&simplex.v1, &simplex.v2, &simplex.v3}
	simplex.count = cache.count
	for i := range simplex.count {
		v := vertices[i]
		v.indexA = cache.indexA[i]
		v.indexB = cache.indexB[i]
		v.wA = proxyA.points[v.indexA]
		v.wB = localB.points[v.indexB]
		v.w = v.wA.sub(v.wB)
		v.a = -1
	}
	if simplex.count == 0 {
		simplex.count = 1
		simplex.v1.wA = proxyA.points[0]
		simplex.v1.wB = localB.points[0]
		simplex.v1.w = simplex.v1.wA.sub(simplex.v1.wB)
		simplex.v1.a = 1
	}
	var nonUnitNormal f64Vec
	var saveA, saveB [3]int

	iteration := 0
	for iteration < 20 {
		saveCount := simplex.count
		for i := range saveCount {
			saveA[i] = vertices[i].indexA
			saveB[i] = vertices[i].indexB
		}

		var d f64Vec
		switch simplex.count {
		case 1:
			d = simplex.v1.w.neg()
		case 2:
			d = f64SolveSimplex2(&simplex)
		default:
			d = f64SolveSimplex3(&simplex)
		}

		if simplex.count == 3 {
			a, b := f64WitnessPoints(&simplex)
			output.pointA = xfA.apply(a)
			output.pointB = xfA.apply(b)
			return output
		}

		if d.dot(d) < 1.1920929e-7*1.1920929e-7 {
			a, b := f64WitnessPoints(&simplex)
			output.pointA = xfA.apply(a)
			output.pointB = xfA.apply(b)
			return output
		}

		nonUnitNormal = d

		vertex := vertices[simplex.count]
		vertex.indexA = f64FindSupport(proxyA, d)
		vertex.wA = proxyA.points[vertex.indexA]
		vertex.indexB = f64FindSupport(&localB, d.neg())
		vertex.wB = localB.points[vertex.indexB]
		vertex.w = vertex.wA.sub(vertex.wB)

		iteration++

		duplicate := false
		for i := range saveCount {
			if vertex.indexA == saveA[i] && vertex.indexB == saveB[i] {
				duplicate = true
				break
			}
		}
		if duplicate {
			break
		}

		simplex.count++
	}

	length := math.Sqrt(nonUnitNormal.dot(nonUnitNormal))
	normal := f64Vec{nonUnitNormal.x / length, nonUnitNormal.y / length}
	normal = xfA.rotate(normal)

	a, b := f64WitnessPoints(&simplex)
	delta := a.sub(b)
	output.normal = normal
	output.distance = math.Sqrt(delta.dot(delta))
	output.pointA = xfA.apply(a)
	output.pointB = xfA.apply(b)
	output.iterations = iteration

	cache.count = simplex.count
	for k := range simplex.count {
		cache.indexA[k] = vertices[k].indexA
		cache.indexB[k] = vertices[k].indexB
	}

	if useRadii && output.distance > 0.1*0.005 {
		rA, rB := proxyA.radius, proxyB.radius
		output.distance = math.Max(0, output.distance-rA-rB)
		output.pointA = output.pointA.add(f64Vec{rA * normal.x, rA * normal.y})
		output.pointB = output.pointB.sub(f64Vec{rB * normal.x, rB * normal.y})
	}

	return output
}

func qToF64(q Q) float64 { return float64(q.Raw()) / 4294967296.0 }

func proxyToF64(p *ShapeProxy) f64Proxy {
	var out f64Proxy
	out.count = p.Count
	out.radius = qToF64(p.Radius)
	for i := range p.Count {
		out.points[i] = f64Vec{qToF64(p.Points[i].X), qToF64(p.Points[i].Y)}
	}
	return out
}

func transformToF64(t Transform) f64Transform {
	return f64Transform{qToF64(t.P.X), qToF64(t.P.Y), qToF64(t.Q.Cos), qToF64(t.Q.Sin)}
}

// randomProxy builds a rotated box, a capsule or a circle proxy with
// coordinates on a millimetre grid.
func randomProxy(rng *rand.Rand) ShapeProxy {
	milli := func(lo, hi int) Q { return fixed.Q32FromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	switch rng.Intn(3) {
	case 0:
		box := MakeBox(milli(100, 2000), milli(100, 2000))
		return MakeProxy(box.Vertices[:box.Count], fixed.Q32Zero())
	case 1:
		points := []Vec2{{X: milli(-1000, 1000), Y: milli(-1000, 1000)}, {X: milli(-1000, 1000), Y: milli(-1000, 1000)}}
		return MakeProxy(points, milli(50, 500))
	default:
		return MakeProxy([]Vec2{{X: milli(-500, 500), Y: milli(-500, 500)}}, milli(50, 1000))
	}
}

// randomTransform places a proxy inside a ten metre box at a random angle
// on a grid of a thousandth of a turn.
func randomTransform(rng *rand.Rand) Transform {
	milli := func(lo, hi int) Q { return fixed.Q32FromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	return Transform{
		P: Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
		Q: MakeRot(milli(0, 999)),
	}
}

// TestShapeDistanceTracksTheFloat64Mirror keeps the Q solver within a
// micrometre of the float64 formulation on random pairs. Pairs where the
// two disagree on overlap are skipped: a tie in a branch is a legitimate
// difference of the grid, not a defect.
func TestShapeDistanceTracksTheFloat64Mirror(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	limit := math.Ldexp(1, -20)
	compared := 0

	for range 1000 {
		input := DistanceInput{
			ProxyA:     randomProxy(rng),
			ProxyB:     randomProxy(rng),
			TransformA: randomTransform(rng),
			TransformB: randomTransform(rng),
			UseRadii:   true,
		}
		var cache SimplexCache
		got := ShapeDistance(&input, &cache, nil)

		pa, pb := proxyToF64(&input.ProxyA), proxyToF64(&input.ProxyB)
		want := shapeDistanceF64(&pa, &pb, transformToF64(input.TransformA), transformToF64(input.TransformB), true, &f64SimplexCache{})

		if (want.distance == 0) != got.Distance.Eq(fixed.Q32Zero()) {
			continue
		}
		compared++

		if diff := math.Abs(qToF64(got.Distance) - want.distance); diff > limit {
			t.Fatalf("distance differs by %g (Q %v, float %g)", diff, got.Distance, want.distance)
		}
	}

	if compared < 900 {
		t.Fatalf("only %d pairs were comparable", compared)
	}
}

// The float64 mirror of the time of impact follows the TOI half of
// distance.go line by line, with the same cache hand-off to the mirror of
// the GJK solver.

type f64Sweep struct {
	localCenter, c1, c2 f64Vec
	q1c, q1s, q2c, q2s  float64
}

func sweepToF64(s *Sweep) f64Sweep {
	v := func(p Vec2) f64Vec { return f64Vec{qToF64(p.X), qToF64(p.Y)} }
	return f64Sweep{
		localCenter: v(s.LocalCenter),
		c1:          v(s.C1),
		c2:          v(s.C2),
		q1c:         qToF64(s.Q1.Cos), q1s: qToF64(s.Q1.Sin),
		q2c: qToF64(s.Q2.Cos), q2s: qToF64(s.Q2.Sin),
	}
}

func sweepTransformF64(s *f64Sweep, t float64) f64Transform {
	omt := 1 - t
	px := omt*s.c1.x + t*s.c2.x
	py := omt*s.c1.y + t*s.c2.y
	qc := omt*s.q1c + t*s.q2c
	qs := omt*s.q1s + t*s.q2s
	mag := math.Sqrt(qc*qc + qs*qs)
	if mag > 0 {
		qc, qs = qc/mag, qs/mag
	}
	xf := f64Transform{px: px, py: py, qc: qc, qs: qs}
	shift := xf.rotate(s.localCenter)
	xf.px -= shift.x
	xf.py -= shift.y
	return xf
}

func (t f64Transform) invRotate(v f64Vec) f64Vec {
	return f64Vec{t.qc*v.x + t.qs*v.y, -t.qs*v.x + t.qc*v.y}
}

type f64Separation struct {
	proxyA, proxyB *f64Proxy
	sweepA, sweepB f64Sweep
	localPoint     f64Vec
	axis           f64Vec
	kind           separationType
}

func f64Normalize(v f64Vec) f64Vec {
	length := math.Sqrt(v.dot(v))
	return f64Vec{v.x / length, v.y / length}
}

func makeSeparationF64(cache *f64SimplexCache, proxyA *f64Proxy, sweepA *f64Sweep, proxyB *f64Proxy, sweepB *f64Sweep, t1 float64) f64Separation {
	f := f64Separation{proxyA: proxyA, proxyB: proxyB, sweepA: *sweepA, sweepB: *sweepB}
	xfA := sweepTransformF64(sweepA, t1)
	xfB := sweepTransformF64(sweepB, t1)

	if cache.count == 1 {
		f.kind = pointsType
		pointA := xfA.apply(proxyA.points[cache.indexA[0]])
		pointB := xfB.apply(proxyB.points[cache.indexB[0]])
		f.axis = f64Normalize(pointB.sub(pointA))
		return f
	}

	if cache.indexA[0] == cache.indexA[1] {
		f.kind = faceBType
		b1 := proxyB.points[cache.indexB[0]]
		b2 := proxyB.points[cache.indexB[1]]
		edge := b2.sub(b1)
		f.axis = f64Normalize(f64Vec{edge.y, -edge.x})
		normal := xfB.rotate(f.axis)
		f.localPoint = f64Vec{0.5 * (b1.x + b2.x), 0.5 * (b1.y + b2.y)}
		pointB := xfB.apply(f.localPoint)
		pointA := xfA.apply(proxyA.points[cache.indexA[0]])
		if pointA.sub(pointB).dot(normal) < 0 {
			f.axis = f.axis.neg()
		}
		return f
	}

	f.kind = faceAType
	a1 := proxyA.points[cache.indexA[0]]
	a2 := proxyA.points[cache.indexA[1]]
	edge := a2.sub(a1)
	f.axis = f64Normalize(f64Vec{edge.y, -edge.x})
	normal := xfA.rotate(f.axis)
	f.localPoint = f64Vec{0.5 * (a1.x + a2.x), 0.5 * (a1.y + a2.y)}
	pointA := xfA.apply(f.localPoint)
	pointB := xfB.apply(proxyB.points[cache.indexB[0]])
	if pointB.sub(pointA).dot(normal) < 0 {
		f.axis = f.axis.neg()
	}
	return f
}

func findMinSeparationF64(f *f64Separation, t float64) (float64, int, int) {
	xfA := sweepTransformF64(&f.sweepA, t)
	xfB := sweepTransformF64(&f.sweepB, t)
	switch f.kind {
	case pointsType:
		indexA := f64FindSupport(f.proxyA, xfA.invRotate(f.axis))
		indexB := f64FindSupport(f.proxyB, xfB.invRotate(f.axis.neg()))
		pointA := xfA.apply(f.proxyA.points[indexA])
		pointB := xfB.apply(f.proxyB.points[indexB])
		return pointB.sub(pointA).dot(f.axis), indexA, indexB
	case faceAType:
		normal := xfA.rotate(f.axis)
		pointA := xfA.apply(f.localPoint)
		indexB := f64FindSupport(f.proxyB, xfB.invRotate(normal.neg()))
		pointB := xfB.apply(f.proxyB.points[indexB])
		return pointB.sub(pointA).dot(normal), -1, indexB
	default:
		normal := xfB.rotate(f.axis)
		pointB := xfB.apply(f.localPoint)
		indexA := f64FindSupport(f.proxyA, xfA.invRotate(normal.neg()))
		pointA := xfA.apply(f.proxyA.points[indexA])
		return pointA.sub(pointB).dot(normal), indexA, -1
	}
}

func evaluateSeparationF64(f *f64Separation, indexA, indexB int, t float64) float64 {
	xfA := sweepTransformF64(&f.sweepA, t)
	xfB := sweepTransformF64(&f.sweepB, t)
	switch f.kind {
	case pointsType:
		pointA := xfA.apply(f.proxyA.points[indexA])
		pointB := xfB.apply(f.proxyB.points[indexB])
		return pointB.sub(pointA).dot(f.axis)
	case faceAType:
		normal := xfA.rotate(f.axis)
		pointA := xfA.apply(f.localPoint)
		pointB := xfB.apply(f.proxyB.points[indexB])
		return pointB.sub(pointA).dot(normal)
	default:
		normal := xfB.rotate(f.axis)
		pointB := xfB.apply(f.localPoint)
		pointA := xfA.apply(f.proxyA.points[indexA])
		return pointA.sub(pointB).dot(normal)
	}
}

type f64TOIOutput struct {
	state    TOIState
	fraction float64
}

// timeOfImpactF64 mirrors TimeOfImpact.
func timeOfImpactF64(proxyA, proxyB *f64Proxy, sweepA, sweepB f64Sweep, maxFraction float64) f64TOIOutput {
	const slop = 0.005
	output := f64TOIOutput{state: TOIStateUnknown, fraction: maxFraction}

	tMax := maxFraction
	target := math.Max(slop, proxyA.radius+proxyB.radius-slop)
	tolerance := 0.25 * slop

	t1 := 0.0
	distanceIterations := 0
	var cache f64SimplexCache

	for {
		xfA := sweepTransformF64(&sweepA, t1)
		xfB := sweepTransformF64(&sweepB, t1)
		distanceOutput := shapeDistanceF64(proxyA, proxyB, xfA, xfB, false, &cache)
		distanceIterations++

		if distanceOutput.distance <= 0 {
			output.state = TOIStateOverlapped
			output.fraction = 0
			break
		}
		if distanceOutput.distance <= target+tolerance {
			output.state = TOIStateHit
			output.fraction = t1
			break
		}

		fcn := makeSeparationF64(&cache, proxyA, &sweepA, proxyB, &sweepB, t1)

		done := false
		t2 := tMax
		pushBackIterations := 0
		for {
			s2, indexA, indexB := findMinSeparationF64(&fcn, t2)
			if s2 > target+tolerance {
				output.state = TOIStateSeparated
				output.fraction = tMax
				done = true
				break
			}
			if s2 > target-tolerance {
				t1 = t2
				break
			}
			s1 := evaluateSeparationF64(&fcn, indexA, indexB, t1)
			if s1 < target-tolerance {
				output.state = TOIStateFailed
				output.fraction = t1
				done = true
				break
			}
			if s1 <= target+tolerance {
				output.state = TOIStateHit
				output.fraction = t1
				done = true
				break
			}

			rootIterationCount := 0
			a1, a2 := t1, t2
			for {
				var t float64
				if rootIterationCount&1 == 1 {
					t = a1 + (target-s1)*(a2-a1)/(s2-s1)
				} else {
					t = 0.5 * (a1 + a2)
				}
				rootIterationCount++
				s := evaluateSeparationF64(&fcn, indexA, indexB, t)
				if math.Abs(s-target) < tolerance {
					t2 = t
					break
				}
				if s > target {
					a1, s1 = t, s
				} else {
					a2, s2 = t, s
				}
				if rootIterationCount == 50 {
					break
				}
			}
			pushBackIterations++
			if pushBackIterations == MaxPolygonVertices {
				break
			}
		}
		if done {
			break
		}
		if distanceIterations == 20 {
			output.state = TOIStateFailed
			output.fraction = t1
			break
		}
	}
	return output
}

// randomSweep turns at most a quarter turn, so the interpolated rotation
// never collapses to zero.
func randomSweep(rng *rand.Rand) Sweep {
	milli := func(lo, hi int) Q { return fixed.Q32FromRatio(lo+rng.Intn(hi-lo+1), 1000) }
	turn := milli(0, 999)
	return Sweep{
		LocalCenter: Vec2{X: milli(-500, 500), Y: milli(-500, 500)},
		C1:          Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
		C2:          Vec2{X: milli(-5000, 5000), Y: milli(-5000, 5000)},
		Q1:          MakeRot(turn),
		Q2:          MakeRot(turn.Add(milli(-250, 250))),
	}
}

// TestTimeOfImpactTracksTheFloat64Mirror keeps the Q fraction close to the
// float64 formulation on random sweeps. Both solvers stop anywhere inside
// the tolerance band around the target, so the limit adds the time that
// band spans at the closing speed of the pair. Pairs where the two
// disagree on the state are skipped: a tie in a branch is a legitimate
// difference of the grid, not a defect.
func TestTimeOfImpactTracksTheFloat64Mirror(t *testing.T) {
	rng := rand.New(rand.NewSource(13))
	compared := 0

	for range 1000 {
		input := TOIInput{
			ProxyA:      randomProxy(rng),
			ProxyB:      randomProxy(rng),
			SweepA:      randomSweep(rng),
			SweepB:      randomSweep(rng),
			MaxFraction: fixed.Q32One(),
		}
		got := TimeOfImpact(&input)

		pa, pb := proxyToF64(&input.ProxyA), proxyToF64(&input.ProxyB)
		want := timeOfImpactF64(&pa, &pb, sweepToF64(&input.SweepA), sweepToF64(&input.SweepB), 1)

		if got.State != want.state {
			continue
		}
		compared++

		// The band spans half a slop; the speed bounds how fast the gap
		// closes, with the points at most 5 m from the center.
		sa, sb := sweepToF64(&input.SweepA), sweepToF64(&input.SweepB)
		linear := sa.c2.sub(sa.c1).sub(sb.c2.sub(sb.c1))
		speed := math.Sqrt(linear.dot(linear)) + 2*math.Pi*0.25*5*2
		limit := math.Ldexp(1, -16) + 0.5*0.005/speed*4

		if diff := math.Abs(qToF64(got.Fraction) - want.fraction); diff > limit {
			t.Fatalf("fraction differs by %g (Q %v, float %g, state %v)", diff, got.Fraction, want.fraction, got.State)
		}
	}

	if compared < 900 {
		t.Fatalf("only %d pairs were comparable", compared)
	}
}
