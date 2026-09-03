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

// shapeDistanceF64 mirrors ShapeDistance with an empty cache.
func shapeDistanceF64(proxyA, proxyB *f64Proxy, xfA, xfB f64Transform, useRadii bool) f64DistanceOutput {
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
	simplex.count = 1
	simplex.v1.wA = proxyA.points[0]
	simplex.v1.wB = localB.points[0]
	simplex.v1.w = simplex.v1.wA.sub(simplex.v1.wB)
	simplex.v1.a = 1

	vertices := [3]*f64SimplexVertex{&simplex.v1, &simplex.v2, &simplex.v3}
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
		want := shapeDistanceF64(&pa, &pb, transformToF64(input.TransformA), transformToF64(input.TransformB), true)

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
