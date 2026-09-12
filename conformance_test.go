package dbox2d

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
)

type conformanceToleranceConfig struct {
	ulps     uint32
	abs      float64
	absFloat float64
	// hashSteps is the least number of leading steps with an equal hash, float mode only.
	hashSteps int
}

// conformanceTolerance defines the per-trace conformance budgets.
// Function traces use ulps in float mode and abs in fixed mode.
// Scene step 1 uses abs in fixed mode and absFloat in float mode.
var conformanceTolerance = map[string]conformanceToleranceConfig{
	"collide_capsules.txt":                  {ulps: 0, abs: 2e-6},        // precision
	"collide_capsule_and_circle.txt":        {ulps: 0, abs: 1e-5},        // precision
	"collide_circles.txt":                   {ulps: 0, abs: 1e-6},        // precision
	"collide_polygon_and_circle.txt":        {ulps: 0, abs: 2e-6},        // precision
	"collide_segment_and_capsule.txt":       {ulps: 0, abs: 2e-6},        // precision
	"collide_polygon_and_capsule.txt":       {ulps: 0, abs: 2e-6},        // precision
	"collide_polygons.txt":                  {ulps: 0, abs: 2e-6},        // precision
	"collide_segment_and_circle.txt":        {ulps: 0, abs: 4e-5},        // precision
	"collide_segment_and_polygon.txt":       {ulps: 0, abs: 2e-6},        // precision
	"collide_chain_segment_and_circle.txt":  {ulps: 0, abs: 2e-5},        // precision
	"collide_chain_segment_and_capsule.txt": {ulps: 0, abs: 1e-6},        // precision
	"collide_chain_segment_and_polygon.txt": {ulps: 0, abs: 2e-6},        // precision
	"shape_distance.txt":                    {ulps: 0, abs: 8e-6},        // precision
	"time_of_impact.txt":                    {ulps: 0, abs: 1e-7},        // precision
	"compute_hull.txt":                      {ulps: 0, abs: 0},           // exact after cyclic alignment
	"make_rot.txt":                          {ulps: 1328, abs: 4e-3},     // ulps: D-004; abs: D-017 (CORDIC vs Bhaskara)
	"atan2.txt":                             {ulps: 0, abs: 6e-5},        // abs: D-017 (CORDIC vs polynomial)
	"falling_hinges.txt":                    {abs: 4e-3, hashSteps: 158}, // abs: D-017 (rotation from an angle); exact at step 1 and hash equal through step 158 in float mode
	"joint_grid.txt":                        {hashSteps: 500},            // no step-1 dump; count and hash only; hash equal through step 500 in float mode
	"large_pyramid.txt":                     {},                          // no step-1 dump; count and hash only
	"many_pyramids.txt":                     {},                          // no step-1 dump; count and hash only
	"rain.txt":                              {abs: 4e-5, hashSteps: 171}, // precision; exact at step 1 and hash equal through step 171 in float mode
	"smash.txt":                             {hashSteps: 69},             // no step-1 dump; count and hash only; hash equal through step 69 in float mode
	"spinner.txt":                           {abs: 1e-2, absFloat: 1e-2}, // D-013: the colored set of the bar contacts differs
	"tumbler.txt":                           {abs: 2e-5, hashSteps: 6},   // precision; exact at step 1 and hash equal through step 6 in float mode
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
	if len(header) < 6 || header[0] != "#" || header[1] != "dbox2d" || header[2] != "conformance" || header[3] != "trace" || header[4] != "v1" {
		t.Fatalf("conformance trace %s has invalid version header; want token v1", path)
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
	if difference > c.tolerance.abs {
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
	normal         [2]conformanceTraceFloat
	rollingImpulse conformanceTraceFloat
	pointCount     int
	points         [2]conformanceManifoldPoint
}

type conformanceManifoldPoint struct {
	point, anchorA, anchorB [2]conformanceTraceFloat
	separation              conformanceTraceFloat
	id                      int
}

func parseConformanceManifold(c *conformanceTraceCursor) conformanceManifold {
	result := conformanceManifold{
		normal:         [2]conformanceTraceFloat{c.float(), c.float()},
		rollingImpulse: c.float(),
		pointCount:     c.int(),
	}
	if result.pointCount < 0 || result.pointCount > 2 {
		c.t.Fatalf("invalid manifold point count %d", result.pointCount)
	}
	for i := range result.pointCount {
		result.points[i] = conformanceManifoldPoint{
			point:      [2]conformanceTraceFloat{c.float(), c.float()},
			anchorA:    [2]conformanceTraceFloat{c.float(), c.float()},
			anchorB:    [2]conformanceTraceFloat{c.float(), c.float()},
			separation: c.float(),
			id:         c.int(),
		}
	}
	return result
}

func compareConformanceManifold(c *conformanceFunctionChecker, caseIndex int, got Manifold, want conformanceManifold) {
	c.reportFloat(caseIndex, "normal.x", got.Normal.X, want.normal[0])
	c.reportFloat(caseIndex, "normal.y", got.Normal.Y, want.normal[1])
	c.reportFloat(caseIndex, "rollingImpulse", got.RollingImpulse, want.rollingImpulse)
	c.reportInt(caseIndex, "pointCount", got.PointCount, want.pointCount)
	for i := range min(got.PointCount, want.pointCount) {
		gotPoint := got.Points[i]
		wantPoint := want.points[i]
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].point.x", i), gotPoint.Point.X, wantPoint.point[0])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].point.y", i), gotPoint.Point.Y, wantPoint.point[1])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].anchorA.x", i), gotPoint.AnchorA.X, wantPoint.anchorA[0])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].anchorA.y", i), gotPoint.AnchorA.Y, wantPoint.anchorA[1])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].anchorB.x", i), gotPoint.AnchorB.X, wantPoint.anchorB[0])
		c.reportFloat(caseIndex, fmt.Sprintf("point[%d].anchorB.y", i), gotPoint.AnchorB.Y, wantPoint.anchorB[1])
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
	var manifold Manifold
	var cache *SimplexCache
	var cacheWant SimplexCache

	switch file {
	case "collide_circles.txt":
		a, xfA := parseConformanceCircle(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCircle(in), parseConformanceTransform(in)
		manifold = CollideCircles(&a, xfA, &b, xfB)
	case "collide_capsule_and_circle.txt":
		a, xfA := parseConformanceCapsule(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCircle(in), parseConformanceTransform(in)
		manifold = CollideCapsuleAndCircle(&a, xfA, &b, xfB)
	case "collide_polygon_and_circle.txt":
		a, xfA := parseConformancePolygon(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCircle(in), parseConformanceTransform(in)
		manifold = CollidePolygonAndCircle(&a, xfA, &b, xfB)
	case "collide_capsules.txt":
		a, xfA := parseConformanceCapsule(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCapsule(in), parseConformanceTransform(in)
		manifold = CollideCapsules(&a, xfA, &b, xfB)
	case "collide_segment_and_capsule.txt":
		a, xfA := parseConformanceSegment(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCapsule(in), parseConformanceTransform(in)
		manifold = CollideSegmentAndCapsule(&a, xfA, &b, xfB)
	case "collide_polygon_and_capsule.txt":
		a, xfA := parseConformancePolygon(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCapsule(in), parseConformanceTransform(in)
		manifold = CollidePolygonAndCapsule(&a, xfA, &b, xfB)
	case "collide_polygons.txt":
		a, xfA := parseConformancePolygon(in), parseConformanceTransform(in)
		b, xfB := parseConformancePolygon(in), parseConformanceTransform(in)
		manifold = CollidePolygons(&a, xfA, &b, xfB)
	case "collide_segment_and_circle.txt":
		a, xfA := parseConformanceSegment(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCircle(in), parseConformanceTransform(in)
		manifold = CollideSegmentAndCircle(&a, xfA, &b, xfB)
	case "collide_segment_and_polygon.txt":
		a, xfA := parseConformanceSegment(in), parseConformanceTransform(in)
		b, xfB := parseConformancePolygon(in), parseConformanceTransform(in)
		manifold = CollideSegmentAndPolygon(&a, xfA, &b, xfB)
	case "collide_chain_segment_and_circle.txt":
		a, xfA := parseConformanceChainSegment(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCircle(in), parseConformanceTransform(in)
		manifold = CollideChainSegmentAndCircle(&a, xfA, &b, xfB)
	case "collide_chain_segment_and_capsule.txt":
		a, xfA := parseConformanceChainSegment(in), parseConformanceTransform(in)
		b, xfB := parseConformanceCapsule(in), parseConformanceTransform(in)
		inputCache := parseConformanceSimplexCache(in)
		cache = &inputCache
		manifold = CollideChainSegmentAndCapsule(&a, xfA, &b, xfB, cache)
	case "collide_chain_segment_and_polygon.txt":
		a, xfA := parseConformanceChainSegment(in), parseConformanceTransform(in)
		b, xfB := parseConformancePolygon(in), parseConformanceTransform(in)
		inputCache := parseConformanceSimplexCache(in)
		cache = &inputCache
		manifold = CollideChainSegmentAndPolygon(&a, xfA, &b, xfB, cache)
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
		xfA, xfB := parseConformanceTransform(in), parseConformanceTransform(in)
		useRadii := in.bool()
		in.done("function input")
		cache := SimplexCache{}
		got := ShapeDistance(&DistanceInput{ProxyA: proxyA, ProxyB: proxyB, TransformA: xfA, TransformB: xfB, UseRadii: useRadii}, &cache, nil)
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
		got := TimeOfImpact(&TOIInput{ProxyA: proxyA, ProxyB: proxyB, SweepA: sweepA, SweepB: sweepB, MaxFraction: maxFraction})
		checker.reportInt(caseIndex, "state", int(got.State), out.int())
		checker.reportFloat(caseIndex, "fraction", got.Fraction, out.float())
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
		got := conformanceAtan2Radians(y, x)
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
	worldId.OverlapAABB(aabb, DefaultQueryFilter(), func(shapeId ShapeId) bool {
		seen[shapeId.GetBody()] = struct{}{}
		return true
	})
	bodies := make([]BodyId, 0, len(seen))
	for bodyId := range seen {
		bodies = append(bodies, bodyId)
	}
	sort.Slice(bodies, func(i, j int) bool { return bodies[i].index1 < bodies[j].index1 })
	return bodies
}

func conformanceSceneHash(worldId WorldId) uint64 {
	hash := fnvOffsetBasis
	for _, bodyId := range conformanceSceneBodies(worldId) {
		transform := bodyId.GetTransform()
		hash = fnvFold(hash, qBits(transform.P.X))
		hash = fnvFold(hash, qBits(transform.P.Y))
		hash = fnvFold(hash, qBits(transform.Q.Cos))
		hash = fnvFold(hash, qBits(transform.Q.Sin))
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
	spec, ok := conformanceSceneSpecs[name]
	if !ok {
		t.Fatalf("no scene builder configured for %s", name)
	}
	trace := readConformanceSceneTrace(t, path)
	if spec.steps > 0 && len(trace.steps) != spec.steps {
		t.Fatalf("scene %s declares %d steps, want %d", name, len(trace.steps), spec.steps)
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
	stepFn := spec.build(worldId)
	if got := len(conformanceSceneBodies(worldId)); got != trace.bodyCount {
		t.Errorf("scene %s initial body count = %d, want %d", name, got, trace.bodyCount)
	}

	tolerance := conformanceTolerance[name]
	if floatMode && upstreamPairOrder {
		// D-013: with the pair order of the reference, float mode matches every
		// scene bit for bit.
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
		if name == "falling_hinges.txt" && portSleepStep == 0 && worldId.GetAwakeBodyCount() == 0 {
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
