package b2_test

// The benchmark scene builders mirror samples/benchmarks.go on purpose: the
// samples module cannot be imported by this library module's tests.

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"testing"

	. "github.com/dhannyell/dbox2d"
)

type conformanceToleranceConfig struct {
	ulps     uint32
	abs      float64
	absFloat float64
	// absHit is the fixed-mode budget for the TOI hit point and normal, which
	// inherit the witness-point error of the final GJK call.
	absHit float64
	// hashSteps is the least number of leading steps with an equal hash, float mode only.
	hashSteps int
}

// conformanceScenesBitExact gates every scene to the exact hash in float
// mode, through the whole trace.
const conformanceScenesBitExact = true

// conformanceTolerance defines the per-trace conformance budgets.
// Function traces use ulps in float mode and abs in fixed mode.
// Scene step 1 uses abs in fixed mode; float mode matches every scene bit
// for bit (conformanceScenesBitExact).
var conformanceTolerance = map[string]conformanceToleranceConfig{
	"collide_capsules.txt":                  {ulps: 0, abs: 2e-6},               // precision
	"collide_capsule_and_circle.txt":        {ulps: 0, abs: 1e-5},               // precision
	"collide_circles.txt":                   {ulps: 0, abs: 1e-6},               // precision
	"collide_polygon_and_circle.txt":        {ulps: 0, abs: 2e-6},               // precision
	"collide_segment_and_capsule.txt":       {ulps: 0, abs: 2e-6},               // precision
	"collide_polygon_and_capsule.txt":       {ulps: 0, abs: 2e-6},               // precision
	"collide_polygons.txt":                  {ulps: 0, abs: 2e-6},               // precision
	"collide_segment_and_circle.txt":        {ulps: 0, abs: 4e-5},               // precision
	"collide_segment_and_polygon.txt":       {ulps: 0, abs: 2e-6},               // precision
	"collide_chain_segment_and_circle.txt":  {ulps: 0, abs: 2e-5},               // precision
	"collide_chain_segment_and_capsule.txt": {ulps: 0, abs: 1e-6},               // precision
	"collide_chain_segment_and_polygon.txt": {ulps: 0, abs: 2e-6},               // precision
	"shape_distance.txt":                    {ulps: 0, abs: 8e-6},               // precision
	"time_of_impact.txt":                    {ulps: 0, abs: 1e-7, absHit: 2e-6}, // precision
	"compute_hull.txt":                      {ulps: 0, abs: 0},                  // exact after cyclic alignment
	"make_rot.txt":                          {ulps: 1328, abs: 4e-3},            // ulps: D-004; abs: D-017 (CORDIC vs Bhaskara)
	"atan2.txt":                             {ulps: 0, abs: 6e-5},               // abs: D-017 (CORDIC vs polynomial)
	"falling_hinges.txt":                    {abs: 4e-3},                        // abs: D-017 (rotation from an angle)
	"joint_grid.txt":                        {},                                 // no step-1 dump; count and hash only
	"large_pyramid.txt":                     {},                                 // no step-1 dump; count and hash only
	"many_pyramids.txt":                     {},                                 // no step-1 dump; count and hash only
	"rain.txt":                              {abs: 4e-5},                        // precision
	"smash.txt":                             {},                                 // no step-1 dump; count and hash only
	"spinner.txt":                           {},                                 // no step-1 dump; count and hash only
	"tumbler.txt":                           {abs: 2e-5},                        // precision
}

type conformanceTraceFloat struct {
	bits  uint32
	value float32
}

func parseConformanceFloat(t *testing.T, token string) conformanceTraceFloat {
	t.Helper()
	if len(token) != 8 {
		t.Fatalf("conformance float %q is not 8 hexadecimal digits", token)
	}
	bits, err := strconv.ParseUint(token, 16, 32)
	if err != nil {
		t.Fatalf("invalid conformance float %q: %v", token, err)
	}
	return conformanceTraceFloat{bits: uint32(bits), value: math.Float32frombits(uint32(bits))}
}

func (f conformanceTraceFloat) q() Q {
	// Trace inputs are test data, not simulation state.
	return QFromFloat64(float64(f.value))
}

type conformanceTraceCursor struct {
	tokens []string
	pos    int
	t      *testing.T
}

func newConformanceTraceCursor(t *testing.T, tokens []string) *conformanceTraceCursor {
	return &conformanceTraceCursor{tokens: tokens, t: t}
}

func (c *conformanceTraceCursor) take() string {
	if c.pos >= len(c.tokens) {
		c.t.Fatalf("unexpected end of conformance token sequence")
	}
	token := c.tokens[c.pos]
	c.pos++
	return token
}

func (c *conformanceTraceCursor) float() conformanceTraceFloat {
	return parseConformanceFloat(c.t, c.take())
}

func (c *conformanceTraceCursor) int() int {
	value, err := strconv.Atoi(c.take())
	if err != nil {
		c.t.Fatalf("invalid conformance integer: %v", err)
	}
	return value
}

func (c *conformanceTraceCursor) bool() bool {
	value := c.int()
	if value != 0 && value != 1 {
		c.t.Fatalf("invalid conformance boolean %d", value)
	}
	return value == 1
}

func (c *conformanceTraceCursor) done(context string) {
	if c.pos != len(c.tokens) {
		c.t.Fatalf("%s has %d unexpected trailing tokens", context, len(c.tokens)-c.pos)
	}
}

func readConformanceTraceLines(t *testing.T, path, kind string) []string {
	t.Helper()
	file, err := os.Open(path)
	if err != nil {
		t.Fatalf("open conformance trace %s: %v", path, err)
	}
	defer func() { _ = file.Close() }()

	scanner := bufio.NewScanner(file)
	scanner.Buffer(make([]byte, 1024), 16<<20)
	if !scanner.Scan() {
		if err := scanner.Err(); err != nil {
			t.Fatalf("read conformance trace %s: %v", path, err)
		}
		t.Fatalf("conformance trace %s is empty", path)
	}
	header := strings.Fields(scanner.Text())
	if len(header) < 6 || header[0] != "#" || header[1] != "dbox2d" || header[2] != "conformance" || header[3] != "trace" || (header[4] != "v1" && header[4] != "v2") {
		t.Fatalf("conformance trace %s has invalid version header; want token v1 or v2", path)
	}
	gotKind := ""
	for _, token := range header[5:] {
		if strings.HasPrefix(token, "kind=") {
			gotKind = strings.TrimPrefix(token, "kind=")
			break
		}
	}
	if gotKind != kind {
		t.Fatalf("conformance trace %s has kind=%q, want kind=%q", path, gotKind, kind)
	}

	var lines []string
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line != "" {
			lines = append(lines, line)
		}
	}
	if err := scanner.Err(); err != nil {
		t.Fatalf("read conformance trace %s: %v", path, err)
	}
	return lines
}

type conformanceFunctionCase struct {
	index  int
	input  []string
	output []string
}

func readConformanceFunctionTrace(t *testing.T, path string) []conformanceFunctionCase {
	lines := readConformanceTraceLines(t, path, "function")
	if len(lines) == 0 {
		t.Fatalf("function trace %s has no cases line", path)
	}
	header := strings.Fields(lines[0])
	if len(header) != 2 || header[0] != "cases" {
		t.Fatalf("function trace %s has invalid cases line", path)
	}
	count, err := strconv.Atoi(header[1])
	if err != nil || count < 0 {
		t.Fatalf("function trace %s has invalid case count %q", path, header[1])
	}
	if len(lines[1:]) != count {
		t.Fatalf("function trace %s declares %d cases but contains %d", path, count, len(lines[1:]))
	}

	cases := make([]conformanceFunctionCase, count)
	for i := range count {
		fields := strings.Fields(lines[i+1])
		if len(fields) < 3 || fields[0] != "case" {
			t.Fatalf("function trace %s case %d has invalid line", path, i)
		}
		caseIndex, err := strconv.Atoi(fields[1])
		if err != nil || caseIndex != i {
			t.Fatalf("function trace %s case line %d has index %q", path, i, fields[1])
		}
		separator := -1
		for offset, token := range fields[2:] {
			j := offset + 2
			if token == "|" {
				if separator != -1 {
					t.Fatalf("function trace %s case %d has more than one |", path, i)
				}
				separator = j
			}
		}
		if separator == -1 {
			t.Fatalf("function trace %s case %d has no | separator", path, i)
		}
		cases[i] = conformanceFunctionCase{
			index:  caseIndex,
			input:  append([]string(nil), fields[2:separator]...),
			output: append([]string(nil), fields[separator+1:]...),
		}
	}
	return cases
}

type conformanceSceneBody struct {
	position [2]conformanceTraceFloat
	rotation [2]conformanceTraceFloat
}

type conformanceSceneStep struct {
	index  int
	hash   uint64
	bodies map[int]conformanceSceneBody
}

type conformanceSceneTrace struct {
	bodyCount int
	steps     []conformanceSceneStep
}

func parseConformanceHash(t *testing.T, token string) uint64 {
	t.Helper()
	if len(token) != 16 {
		t.Fatalf("conformance hash %q is not 16 hexadecimal digits", token)
	}
	hash, err := strconv.ParseUint(token, 16, 64)
	if err != nil {
		t.Fatalf("invalid conformance hash %q: %v", token, err)
	}
	return hash
}

func readConformanceSceneTrace(t *testing.T, path string) conformanceSceneTrace {
	lines := readConformanceTraceLines(t, path, "scene")
	if len(lines) < 2 {
		t.Fatalf("scene trace %s has no bodies/steps header", path)
	}
	readCount := func(line, label string) int {
		fields := strings.Fields(line)
		if len(fields) != 2 || fields[0] != label {
			t.Fatalf("scene trace %s has invalid %s line", path, label)
		}
		value, err := strconv.Atoi(fields[1])
		if err != nil || value < 0 {
			t.Fatalf("scene trace %s has invalid %s count %q", path, label, fields[1])
		}
		return value
	}
	bodyCount := readCount(lines[0], "bodies")
	stepCount := readCount(lines[1], "steps")
	trace := conformanceSceneTrace{bodyCount: bodyCount, steps: make([]conformanceSceneStep, 0, stepCount)}
	var current *conformanceSceneStep
	for _, line := range lines[2:] {
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}
		switch fields[0] {
		case "step":
			if len(fields) != 3 {
				t.Fatalf("scene trace %s has invalid step line", path)
			}
			index, err := strconv.Atoi(fields[1])
			if err != nil || index != len(trace.steps)+1 {
				t.Fatalf("scene trace %s has step index %q, want %d", path, fields[1], len(trace.steps)+1)
			}
			trace.steps = append(trace.steps, conformanceSceneStep{index: index, hash: parseConformanceHash(t, fields[2]), bodies: make(map[int]conformanceSceneBody)})
			current = &trace.steps[len(trace.steps)-1]
		case "body":
			if current == nil || len(fields) != 6 {
				t.Fatalf("scene trace %s has body line before a step or invalid fields", path)
			}
			index, err := strconv.Atoi(fields[1])
			if err != nil || index < 0 {
				t.Fatalf("scene trace %s has invalid body index %q", path, fields[1])
			}
			if _, exists := current.bodies[index]; exists {
				t.Fatalf("scene trace %s repeats body %d at step %d", path, index, current.index)
			}
			current.bodies[index] = conformanceSceneBody{
				position: [2]conformanceTraceFloat{parseConformanceFloat(t, fields[2]), parseConformanceFloat(t, fields[3])},
				rotation: [2]conformanceTraceFloat{parseConformanceFloat(t, fields[4]), parseConformanceFloat(t, fields[5])},
			}
		default:
			t.Fatalf("scene trace %s has unknown line %q", path, line)
		}
	}
	if len(trace.steps) != stepCount {
		t.Fatalf("scene trace %s declares %d steps but contains %d", path, stepCount, len(trace.steps))
	}
	return trace
}

func conformanceFloatMode() bool {
	return ScalarMode == "float"
}

func orderedConformanceFloat(bits uint32) uint32 {
	if bits&0x80000000 != 0 {
		return ^bits + 1
	}
	return bits | 0x80000000
}

func conformanceULPDistance(a, b uint32) uint32 {
	if a == 0x80000000 {
		a = 0
	}
	if b == 0x80000000 {
		b = 0
	}
	oa, ob := orderedConformanceFloat(a), orderedConformanceFloat(b)
	if oa > ob {
		return oa - ob
	}
	return ob - oa
}

type conformanceFunctionChecker struct {
	t         *testing.T
	file      string
	tolerance conformanceToleranceConfig
	floatMode bool
	count     int
	maxULP    uint32
	maxAbs    float64
	exceeded  bool
}

func (c *conformanceFunctionChecker) reportFloat(caseIndex int, field string, got Q, want conformanceTraceFloat) {
	if c.floatMode {
		gotBits := math.Float32bits(float32(QToFloat64(got)))
		distance := conformanceULPDistance(gotBits, want.bits)
		if distance == 0 {
			return
		}
		c.count++
		if distance > c.maxULP {
			c.maxULP = distance
		}
		message := fmt.Sprintf("%s case %d field %s: got 0x%08x (%g), want 0x%08x (%g), ulps=%d", c.file, caseIndex, field, gotBits, float32(QToFloat64(got)), want.bits, want.value, distance)
		if distance > c.tolerance.ulps {
			c.exceeded = true
			c.t.Errorf("%s", message)
		}
		return
	}

	gotValue := QToFloat64(got)
	difference := math.Abs(gotValue - float64(want.value))
	if difference == 0 {
		return
	}
	c.count++
	if difference > c.maxAbs {
		c.maxAbs = difference
	}
	message := fmt.Sprintf("%s case %d field %s: got 0x%016x (%g), want 0x%08x (%g), abs=%g", c.file, caseIndex, field, qBits(got), gotValue, want.bits, want.value, difference)
	budget := c.tolerance.abs
	if c.tolerance.absHit > 0 && (strings.HasPrefix(field, "point.") || strings.HasPrefix(field, "normal.")) {
		budget = c.tolerance.absHit
	}
	if difference > budget {
		c.exceeded = true
		c.t.Errorf("%s", message)
	}
}

func (c *conformanceFunctionChecker) reportInt(caseIndex int, field string, got, want int) {
	if got == want {
		return
	}
	// Fixed-point distance iterations can vary while the numeric trace stays within its budget.
	if c.file == "shape_distance.txt" && field == "iterations" && !c.floatMode {
		return
	}
	// v3.2 writes the simplex cache on every exit; in fixed mode a near tie
	// can pick another witness vertex while the manifold stays within budget.
	if strings.HasPrefix(c.file, "collide_chain_segment_and_") && strings.HasPrefix(field, "cache.") && !c.floatMode {
		return
	}
	c.count++
	c.exceeded = true
	c.t.Errorf("%s case %d field %s: got %d, want %d", c.file, caseIndex, field, got, want)
}

func (c *conformanceFunctionChecker) finish() {
	if c.count == 0 {
		return
	}
	if c.floatMode {
		c.t.Logf("%s: total mismatches=%d, max ulps=%d", c.file, c.count, c.maxULP)
	} else {
		c.t.Logf("%s: total mismatches=%d, max abs diff=%g", c.file, c.count, c.maxAbs)
	}
}

type conformanceManifold struct {
	normal     [2]conformanceTraceFloat
	pointCount int
	points     [2]conformanceManifoldPoint
}

// conformanceManifoldPoint is one point of a format v2 local manifold, in
// the frame of shape A.
type conformanceManifoldPoint struct {
	point      [2]conformanceTraceFloat
	separation conformanceTraceFloat
	id         int
}

func parseConformanceManifold(c *conformanceTraceCursor) conformanceManifold {
	result := conformanceManifold{
		normal:     [2]conformanceTraceFloat{c.float(), c.float()},
		pointCount: c.int(),
	}
	if result.pointCount < 0 || result.pointCount > 2 {
		c.t.Fatalf("invalid manifold point count %d", result.pointCount)
	}
	for i := range result.pointCount {
		result.points[i] = conformanceManifoldPoint{
			point:      [2]conformanceTraceFloat{c.float(), c.float()},
			separation: c.float(),
			id:         c.int(),
		}
	}
	return result
}

func compareConformanceManifold(c *conformanceFunctionChecker, caseIndex int, got LocalManifold, want conformanceManifold) {
	c.reportFloat(caseIndex, "normal.x", got.Normal.X, want.normal[0])
	c.reportFloat(caseIndex, "normal.y", got.Normal.Y, want.normal[1])
	c.reportInt(caseIndex, "pointCount", got.PointCount, want.pointCount)
	for i := range min(got.PointCount, want.pointCount) {
		gotPoint := got.Points[i]
		wantPoint := want.points[i]
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].point.x", i), gotPoint.Point.X, wantPoint.point[0])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].point.y", i), gotPoint.Point.Y, wantPoint.point[1])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].separation", i), gotPoint.Separation, wantPoint.separation)
		c.reportInt(caseIndex, fmt.Sprintf("point[%d].id", i), int(gotPoint.Id), wantPoint.id)
	}
}

func parseConformanceSimplexCache(c *conformanceTraceCursor) SimplexCache {
	result := SimplexCache{Count: uint16(c.int())}
	for i := range result.IndexA {
		result.IndexA[i] = uint8(c.int())
	}
	for i := range result.IndexB {
		result.IndexB[i] = uint8(c.int())
	}
	return result
}

func compareConformanceSimplexCache(c *conformanceFunctionChecker, caseIndex int, got, want SimplexCache) {
	c.reportInt(caseIndex, "cache.count", int(got.Count), int(want.Count))
	for i := range got.IndexA {
		c.reportInt(caseIndex, fmt.Sprintf("cache.indexA[%d]", i), int(got.IndexA[i]), int(want.IndexA[i]))
		c.reportInt(caseIndex, fmt.Sprintf("cache.indexB[%d]", i), int(got.IndexB[i]), int(want.IndexB[i]))
	}
}

func parseConformanceVec2(c *conformanceTraceCursor) Vec2 {
	return Vec2{X: c.float().q(), Y: c.float().q()}
}

func parseConformanceRot(c *conformanceTraceCursor) Rot {
	cosine, sine := c.float().q(), c.float().q()
	return Rot{Cos: cosine, Sin: sine}
}

func parseConformanceTransform(c *conformanceTraceCursor) Transform {
	return Transform{P: parseConformanceVec2(c), Q: parseConformanceRot(c)}
}

func parseConformanceCircle(c *conformanceTraceCursor) Circle {
	return Circle{Center: parseConformanceVec2(c), Radius: c.float().q()}
}

func parseConformanceCapsule(c *conformanceTraceCursor) Capsule {
	return Capsule{Center1: parseConformanceVec2(c), Center2: parseConformanceVec2(c), Radius: c.float().q()}
}

func parseConformanceSegment(c *conformanceTraceCursor) Segment {
	return Segment{Point1: parseConformanceVec2(c), Point2: parseConformanceVec2(c)}
}

func parseConformanceChainSegment(c *conformanceTraceCursor) ChainSegment {
	return ChainSegment{
		Ghost1:  parseConformanceVec2(c),
		Segment: parseConformanceSegment(c),
		Ghost2:  parseConformanceVec2(c),
		ChainId: c.int(),
	}
}

func parseConformancePolygon(c *conformanceTraceCursor) Polygon {
	result := Polygon{Count: c.int()}
	if result.Count < 0 || result.Count > MaxPolygonVertices {
		c.t.Fatalf("invalid polygon count %d", result.Count)
	}
	for i := range result.Vertices {
		result.Vertices[i] = parseConformanceVec2(c)
	}
	for i := range result.Normals {
		result.Normals[i] = parseConformanceVec2(c)
	}
	result.Centroid = parseConformanceVec2(c)
	result.Radius = c.float().q()
	return result
}

func parseConformanceProxy(c *conformanceTraceCursor) ShapeProxy {
	result := ShapeProxy{Count: c.int()}
	if result.Count < 0 || result.Count > MaxPolygonVertices {
		c.t.Fatalf("invalid shape proxy count %d", result.Count)
	}
	for i := range result.Points {
		result.Points[i] = parseConformanceVec2(c)
	}
	result.Radius = c.float().q()
	return result
}

func parseConformanceSweep(c *conformanceTraceCursor) Sweep {
	return Sweep{
		LocalCenter: parseConformanceVec2(c),
		C1:          parseConformanceVec2(c),
		C2:          parseConformanceVec2(c),
		Q1:          parseConformanceRot(c),
		Q2:          parseConformanceRot(c),
	}
}

func runConformanceCollisionCase(t *testing.T, file string, caseIndex int, input, output []string, checker *conformanceFunctionChecker) {
	in := newConformanceTraceCursor(t, input)
	out := newConformanceTraceCursor(t, output)
	var manifold LocalManifold
	var cache *SimplexCache
	var cacheWant SimplexCache

	switch file {
	case "collide_circles.txt":
		a, b := parseConformanceCircle(in), parseConformanceCircle(in)
		xf := parseConformanceTransform(in)
		manifold = CollideCircles(&a, &b, xf)
	case "collide_capsule_and_circle.txt":
		a, b := parseConformanceCapsule(in), parseConformanceCircle(in)
		xf := parseConformanceTransform(in)
		manifold = CollideCapsuleAndCircle(&a, &b, xf)
	case "collide_polygon_and_circle.txt":
		a, b := parseConformancePolygon(in), parseConformanceCircle(in)
		xf := parseConformanceTransform(in)
		manifold = CollidePolygonAndCircle(&a, &b, xf)
	case "collide_capsules.txt":
		a, b := parseConformanceCapsule(in), parseConformanceCapsule(in)
		xf := parseConformanceTransform(in)
		manifold = CollideCapsules(&a, &b, xf)
	case "collide_segment_and_capsule.txt":
		a, b := parseConformanceSegment(in), parseConformanceCapsule(in)
		xf := parseConformanceTransform(in)
		manifold = CollideSegmentAndCapsule(&a, &b, xf)
	case "collide_polygon_and_capsule.txt":
		a, b := parseConformancePolygon(in), parseConformanceCapsule(in)
		xf := parseConformanceTransform(in)
		manifold = CollidePolygonAndCapsule(&a, &b, xf)
	case "collide_polygons.txt":
		a, b := parseConformancePolygon(in), parseConformancePolygon(in)
		xf := parseConformanceTransform(in)
		manifold = CollidePolygons(&a, &b, xf)
	case "collide_segment_and_circle.txt":
		a, b := parseConformanceSegment(in), parseConformanceCircle(in)
		xf := parseConformanceTransform(in)
		manifold = CollideSegmentAndCircle(&a, &b, xf)
	case "collide_segment_and_polygon.txt":
		a, b := parseConformanceSegment(in), parseConformancePolygon(in)
		xf := parseConformanceTransform(in)
		manifold = CollideSegmentAndPolygon(&a, &b, xf)
	case "collide_chain_segment_and_circle.txt":
		a, b := parseConformanceChainSegment(in), parseConformanceCircle(in)
		xf := parseConformanceTransform(in)
		manifold = CollideChainSegmentAndCircle(&a, &b, xf)
	case "collide_chain_segment_and_capsule.txt":
		a, b := parseConformanceChainSegment(in), parseConformanceCapsule(in)
		xf := parseConformanceTransform(in)
		inputCache := parseConformanceSimplexCache(in)
		cache = &inputCache
		manifold = CollideChainSegmentAndCapsule(&a, &b, xf, cache)
	case "collide_chain_segment_and_polygon.txt":
		a, b := parseConformanceChainSegment(in), parseConformancePolygon(in)
		xf := parseConformanceTransform(in)
		inputCache := parseConformanceSimplexCache(in)
		cache = &inputCache
		manifold = CollideChainSegmentAndPolygon(&a, &b, xf, cache)
	default:
		t.Fatalf("unsupported collision conformance trace %s", file)
	}
	in.done("function input")
	wantManifold := parseConformanceManifold(out)
	compareConformanceManifold(checker, caseIndex, manifold, wantManifold)
	if cache != nil {
		cacheWant = parseConformanceSimplexCache(out)
		compareConformanceSimplexCache(checker, caseIndex, *cache, cacheWant)
	}
	out.done("function output")
}

func runConformanceFunctionCase(t *testing.T, file string, caseIndex int, input, output []string, checker *conformanceFunctionChecker) {
	if strings.HasPrefix(file, "collide_") {
		runConformanceCollisionCase(t, file, caseIndex, input, output, checker)
		return
	}
	in := newConformanceTraceCursor(t, input)
	out := newConformanceTraceCursor(t, output)
	switch file {
	case "shape_distance.txt":
		proxyA, proxyB := parseConformanceProxy(in), parseConformanceProxy(in)
		xf := parseConformanceTransform(in)
		useRadii := in.bool()
		in.done("function input")
		cache := SimplexCache{}
		got := ComputeShapeDistance(&DistanceInput{ProxyA: proxyA, ProxyB: proxyB, Transform: xf, UseRadii: useRadii}, &cache, nil)
		want := [6]conformanceTraceFloat{out.float(), out.float(), out.float(), out.float(), out.float(), out.float()}
		checker.reportFloat(caseIndex, "normal.x", got.Normal.X, want[0])
		checker.reportFloat(caseIndex, "normal.y", got.Normal.Y, want[1])
		checker.reportFloat(caseIndex, "pointA.x", got.PointA.X, want[2])
		checker.reportFloat(caseIndex, "pointA.y", got.PointA.Y, want[3])
		checker.reportFloat(caseIndex, "pointB.x", got.PointB.X, want[4])
		checker.reportFloat(caseIndex, "pointB.y", got.PointB.Y, want[5])
		checker.reportFloat(caseIndex, "distance", got.Distance, out.float())
		checker.reportInt(caseIndex, "iterations", got.Iterations, out.int())
		checker.reportInt(caseIndex, "simplexCount", got.SimplexCount, out.int())
		out.done("function output")
	case "time_of_impact.txt":
		proxyA, proxyB := parseConformanceProxy(in), parseConformanceProxy(in)
		sweepA, sweepB := parseConformanceSweep(in), parseConformanceSweep(in)
		maxFraction := in.float().q()
		in.done("function input")
		got := ComputeTimeOfImpact(&TOIInput{ProxyA: proxyA, ProxyB: proxyB, SweepA: sweepA, SweepB: sweepB, MaxFraction: maxFraction})
		checker.reportInt(caseIndex, "state", int(got.State), out.int())
		checker.reportFloat(caseIndex, "fraction", got.Fraction, out.float())
		checker.reportFloat(caseIndex, "point.x", got.Point.X, out.float())
		checker.reportFloat(caseIndex, "point.y", got.Point.Y, out.float())
		checker.reportFloat(caseIndex, "normal.x", got.Normal.X, out.float())
		checker.reportFloat(caseIndex, "normal.y", got.Normal.Y, out.float())
		out.done("function output")
	case "compute_hull.txt":
		count := in.int()
		if count < 0 || count > MaxPolygonVertices {
			t.Fatalf("invalid hull input count %d", count)
		}
		points := make([]Vec2, MaxPolygonVertices)
		for i := range points {
			points[i] = parseConformanceVec2(in)
		}
		in.done("function input")
		got := ComputeHull(points[:count])
		wantCount := out.int()
		checker.reportInt(caseIndex, "count", got.Count, wantCount)
		want := [MaxPolygonVertices][2]conformanceTraceFloat{}
		for i := range MaxPolygonVertices {
			want[i] = [2]conformanceTraceFloat{out.float(), out.float()}
		}
		if got.Count == wantCount && wantCount > 0 {
			bestOffset := 0
			bestDifference := math.Inf(1)
			for offset := range wantCount {
				maxDifference := 0.0
				for i := range wantCount {
					gotPoint := got.Points[(i+offset)%wantCount]
					maxDifference = math.Max(maxDifference, math.Abs(QToFloat64(gotPoint.X)-float64(want[i][0].value)))
					maxDifference = math.Max(maxDifference, math.Abs(QToFloat64(gotPoint.Y)-float64(want[i][1].value)))
				}
				if maxDifference < bestDifference {
					bestOffset = offset
					bestDifference = maxDifference
				}
			}
			if bestOffset != 0 {
				t.Logf("compute_hull.txt case %d: hull starts at reference vertex %d", caseIndex, (wantCount-bestOffset)%wantCount)
			}
			for i := range wantCount {
				gotPoint := got.Points[(i+bestOffset)%wantCount]
				checker.reportFloat(caseIndex, fmt.Sprintf("point[%d].x", i), gotPoint.X, want[i][0])
				checker.reportFloat(caseIndex, fmt.Sprintf("point[%d].y", i), gotPoint.Y, want[i][1])
			}
		}
		out.done("function output")
	case "make_rot.txt":
		turns := in.float().q()
		in.done("function input")
		got := MakeRot(turns)
		checker.reportFloat(caseIndex, "cos", got.Cos, out.float())
		checker.reportFloat(caseIndex, "sin", got.Sin, out.float())
		out.done("function output")
	case "atan2.txt":
		y, x := in.float().q(), in.float().q()
		in.done("function input")
		got := Atan2Radians(y, x)
		checker.reportFloat(caseIndex, "radians", got, out.float())
		out.done("function output")
	default:
		t.Fatalf("unsupported function conformance trace %s", file)
	}
}

func TestConformance(t *testing.T) {
	root := filepath.Join("testdata", "conformance")
	if _, err := os.Stat(root); err != nil {
		t.Fatalf("conformance traces are expected in %s: %v", root, err)
	}
	functionPaths, err := filepath.Glob(filepath.Join(root, "functions", "*.txt"))
	if err != nil {
		t.Fatalf("find function conformance traces: %v", err)
	}
	scenePaths, err := filepath.Glob(filepath.Join(root, "scenes", "*.txt"))
	if err != nil {
		t.Fatalf("find scene conformance traces: %v", err)
	}
	if len(functionPaths) == 0 && len(scenePaths) == 0 {
		t.Fatalf("conformance traces are expected in %s/functions and %s/scenes", root, root)
	}

	floatMode := conformanceFloatMode()
	for _, path := range functionPaths {
		name := filepath.Base(path)
		t.Run(name, func(t *testing.T) {
			tolerance, ok := conformanceTolerance[name]
			if !ok {
				t.Fatalf("no conformance tolerance configured for %s", name)
			}
			checker := &conformanceFunctionChecker{t: t, file: name, tolerance: tolerance, floatMode: floatMode}
			for _, testCase := range readConformanceFunctionTrace(t, path) {
				runConformanceFunctionCase(t, name, testCase.index, testCase.input, testCase.output, checker)
			}
			checker.finish()
		})
	}
	for _, path := range scenePaths {
		name := filepath.Base(path)
		t.Run(name, func(t *testing.T) {
			if testing.Short() {
				// Scene traces take minutes on 32-bit and wasm targets; fixed mode is bit-identical there anyway.
				t.Skip("scene traces are skipped in short mode")
			}
			runConformanceSceneTrace(t, name, path, floatMode)
		})
	}
}

func conformanceSceneBodies(worldId WorldId) []BodyId {
	aabb := AABB{LowerBound: Vec2{X: QFromInt(-1000000), Y: QFromInt(-1000000)}, UpperBound: Vec2{X: QFromInt(1000000), Y: QFromInt(1000000)}}
	seen := make(map[BodyId]struct{})
	worldId.OverlapAABB(Vec2{}, aabb, DefaultQueryFilter(), func(shapeId ShapeId) bool {
		seen[shapeId.GetBody()] = struct{}{}
		return true
	})
	bodies := make([]BodyId, 0, len(seen))
	for bodyId := range seen {
		bodies = append(bodies, bodyId)
	}
	sort.Slice(bodies, func(i, j int) bool { return BodyIndex(bodies[i]) < BodyIndex(bodies[j]) })
	return bodies
}

func conformanceSceneHash(worldId WorldId) uint64 {
	hash := FNVOffsetBasis
	for _, bodyId := range conformanceSceneBodies(worldId) {
		transform := bodyId.GetTransform()
		hash = FNVFold(hash, qBits(transform.P.X))
		hash = FNVFold(hash, qBits(transform.P.Y))
		hash = FNVFold(hash, qBits(transform.Q.Cos))
		hash = FNVFold(hash, qBits(transform.Q.Sin))
	}
	return hash
}

type conformanceSceneResidue struct {
	maxAbs float64
	body   int
	field  string
}

func reportConformanceSceneAbs(t *testing.T, tolerance float64, gate bool, step, body int, field string, got Q, want conformanceTraceFloat) float64 {
	difference := math.Abs(QToFloat64(got) - float64(want.value))
	if gate && difference > tolerance {
		t.Errorf("step %d body %d %s mismatch: got 0x%016x (%g), want 0x%08x (%g), abs=%g", step, body, field, qBits(got), QToFloat64(got), want.bits, want.value, difference)
	}
	return difference
}

func runConformanceSceneTrace(t *testing.T, name, path string, floatMode bool) {
	// The trace files carry a .txt suffix; a scene does not.
	spec, ok := conformanceSpecs[strings.TrimSuffix(name, ".txt")]
	if !ok {
		t.Fatalf("no scene builder configured for %s", name)
	}
	trace := readConformanceSceneTrace(t, path)
	if spec.Steps > 0 && len(trace.steps) != spec.Steps {
		t.Fatalf("scene %s declares %d steps, want %d", name, len(trace.steps), spec.Steps)
	}
	if name == "falling_hinges.txt" && len(trace.steps) > 2000 {
		t.Fatalf("falling_hinges trace has %d steps, exceeding the 2000-step cap", len(trace.steps))
	}
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = 1
	worldId := CreateWorld(&worldDef)
	if worldId.IsNull() {
		t.Fatalf("scene %s: CreateWorld returned the null id", name)
	}
	defer DestroyWorld(worldId)
	stepFn := spec.Build(worldId, conformanceSceneOptions(worldId))
	if got := len(conformanceSceneBodies(worldId)); got != trace.bodyCount {
		t.Errorf("scene %s initial body count = %d, want %d", name, got, trace.bodyCount)
	}

	tolerance := conformanceTolerance[name]
	if floatMode && conformanceScenesBitExact {
		// Float mode matches every scene bit for bit.
		tolerance.absFloat = 0
		tolerance.hashSteps = len(trace.steps)
	}
	residues := make([]conformanceSceneResidue, len(trace.steps))
	for i := range residues {
		residues[i].field = "x"
	}
	firstHashDivergence := 0
	portSleepStep := 0
	resetSaturationCount()
	for stepIndex, expected := range trace.steps {
		if stepFn != nil {
			stepFn(stepIndex)
		}
		worldId.Step(QFromRatio(1, 60), 4)
		step := stepIndex + 1
		if got := conformanceSceneHash(worldId); got != expected.hash && firstHashDivergence == 0 {
			firstHashDivergence = step
		}
		if len(expected.bodies) > 0 {
			bodies := conformanceSceneBodies(worldId)
			if len(bodies) != len(expected.bodies) {
				t.Errorf("step %d body count mismatch: got %d, want %d", step, len(bodies), len(expected.bodies))
			} else {
				residue := &residues[stepIndex]
				for bodyIndex, bodyId := range bodies {
					want, ok := expected.bodies[bodyIndex]
					if !ok {
						t.Errorf("step %d missing expected body index %d", step, bodyIndex)
						continue
					}
					transform := bodyId.GetTransform()
					for _, sample := range []struct {
						field string
						got   Q
						want  conformanceTraceFloat
					}{
						{field: "x", got: transform.P.X, want: want.position[0]},
						{field: "y", got: transform.P.Y, want: want.position[1]},
						{field: "c", got: transform.Q.Cos, want: want.rotation[0]},
						{field: "s", got: transform.Q.Sin, want: want.rotation[1]},
					} {
						stepTolerance := tolerance.abs
						if floatMode {
							stepTolerance = tolerance.absFloat
						}
						difference := reportConformanceSceneAbs(t, stepTolerance, step == 1, step, bodyIndex, sample.field, sample.got, sample.want)
						if difference > residue.maxAbs {
							residue.maxAbs = difference
							residue.body = bodyIndex
							residue.field = sample.field
						}
					}
				}
			}
		}
		// The reference marks sleep on the first step without move events,
		// one step after the island falls asleep.
		if name == "falling_hinges.txt" && portSleepStep == 0 && len(worldId.GetBodyEvents().MoveEvents) == 0 {
			portSleepStep = step
		}
	}
	// The contact grid is Q16: a saturation there changes the physics silently.
	if n := saturationCount(); n != 0 {
		t.Errorf("scene %s: %d operations saturated", name, n)
	}
	if name == "falling_hinges.txt" {
		referenceSleepStep := len(trace.steps)
		if portSleepStep != referenceSleepStep {
			t.Logf("falling_hinges: port slept at step %d, reference at %d", portSleepStep, referenceSleepStep)
		}
	}
	if floatMode && tolerance.hashSteps > 0 {
		equalSteps := len(trace.steps)
		if firstHashDivergence != 0 {
			equalSteps = firstHashDivergence - 1
		}
		if equalSteps < tolerance.hashSteps {
			t.Errorf("scene %s: hash equal through step %d, want at least %d", name, equalSteps, tolerance.hashSteps)
		}
	}

	var summary strings.Builder
	fmt.Fprintf(&summary, "scene %s: ", name)
	if len(trace.steps) > 0 && len(trace.steps[0].bodies) > 0 {
		residue := residues[0]
		fmt.Fprintf(&summary, "step 1 max abs=%g (body %d %s)", residue.maxAbs, residue.body, residue.field)
	} else {
		summary.WriteString("step 1 not sampled")
	}
	if firstHashDivergence != 0 {
		fmt.Fprintf(&summary, "; first hash divergence at step %d", firstHashDivergence)
	} else {
		fmt.Fprintf(&summary, "; hash equal through step %d", len(trace.steps))
	}
	for stepIndex := 1; stepIndex < len(trace.steps); stepIndex++ {
		if len(trace.steps[stepIndex].bodies) == 0 {
			continue
		}
		residue := residues[stepIndex]
		fmt.Fprintf(&summary, "; step %d max abs=%g (body %d %s)", stepIndex+1, residue.maxAbs, residue.body, residue.field)
	}
	t.Log(summary.String())
}
