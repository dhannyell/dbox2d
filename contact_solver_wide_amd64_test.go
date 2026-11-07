//go:build dbox2d_simd && !dbox2d_fixed && goexperiment.simd && go1.27 && amd64

package b2

import (
	"math"
	"math/rand"
	"testing"
	"unsafe"
)

// wideKernelCase builds random constraints over random body states. Every
// lane vector gets random values, some of them zeros of both signs,
// infinities and NaN; a lane group of rollingResistance is zero
// in about half the constraints so both relax variants run.
func wideKernelCase(rng *rand.Rand, bodyCount, constraintCount int) ([]contactConstraintWide, []bodyState) {
	special := []float32{0, float32(math.Copysign(0, -1)), float32(math.Inf(1)), float32(math.Inf(-1)), float32(math.NaN()), 1e-38}
	random := func() float32 {
		if rng.Intn(32) == 0 {
			return special[rng.Intn(len(special))]
		}
		return rng.Float32()*4 - 2
	}
	states := make([]bodyState, bodyCount)
	stateFloats := unsafe.Slice((*float32)(unsafe.Pointer(&states[0])), bodyCount*8)
	for i := range stateFloats {
		stateFloats[i] = random()
	}
	constraints := make([]contactConstraintWide, constraintCount)
	size := int(unsafe.Sizeof(contactConstraintWide{}))
	for i := range constraints {
		c := &constraints[i]
		floats := unsafe.Slice((*float32)(unsafe.Pointer(c)), size/4)
		for k := range floats {
			floats[k] = random()
		}
		for j := range wideWidth {
			c.indexA[j], c.indexB[j] = nullIndex, nullIndex
			if rng.Intn(8) != 0 {
				c.indexA[j] = rng.Intn(bodyCount)
			}
			if rng.Intn(8) != 0 {
				c.indexB[j] = rng.Intn(bodyCount)
			}
			c.storeA[j], c.storeB[j] = c.indexA[j], c.indexB[j]
			if rng.Intn(4) == 0 {
				c.storeA[j] = nullIndex
			}
			if rng.Intn(4) == 0 {
				c.storeB[j] = nullIndex
			}
		}
		c.hasRolling = rng.Intn(2) == 0
		rr := laneSlots(&c.rollingResistance, nil)
		for g := 0; g < wideWidth; g += laneGroupWidth {
			if rng.Intn(2) == 0 {
				for j := g; j < g+laneGroupWidth; j++ {
					rr[j] = 0
				}
			}
		}
	}
	return constraints, states
}

func wideFloats[T any](s []T) []float32 {
	var zero T
	return unsafe.Slice((*float32)(unsafe.Pointer(&s[0])), len(s)*int(unsafe.Sizeof(zero))/4)
}

// sameFloats compares bit patterns, except that any two NaNs match: with two
// NaN operands x86 returns the first one, and the Go compiler may swap the
// operands of a commutative operation, so the NaN a lane ends with is not
// fixed by the source.
func sameFloats(a, b []float32) int {
	for i := range a {
		if math.Float32bits(a[i]) != math.Float32bits(b[i]) && !(a[i] != a[i] && b[i] != b[i]) {
			return i
		}
	}
	return -1
}

// TestWideKernelsAsmMatchGo runs each generated kernel and its Go
// counterpart on copies of the same random input and requires identical
// bits in the constraints and the body states, up to the NaN payload. Indices may repeat inside a
// constraint here, which the graph coloring rules out, but both versions
// scatter in the same lane order, so the result is still defined.
func TestWideKernelsAsmMatchGo(t *testing.T) {
	type kernel struct {
		name  string
		goFn  func(int, int, *stepContext, int)
		asmFn func(int, int, *stepContext, int)
	}
	kernels := []kernel{
		{"warmStart", warmStartContactsTaskWide, warmStartContactsTaskWideAsm},
		{"push", pushContactsTaskWide, pushContactsTaskWideAsm},
		{"relax", solveContactsTaskWide, solveContactsTaskWideAsm},
	}
	rng := rand.New(rand.NewSource(7))
	for round := range 500 {
		constraints, states := wideKernelCase(rng, 24, 12)
		for _, k := range kernels {
			run := func(fn func(int, int, *stepContext, int)) ([]float32, []float32) {
				cs := append([]contactConstraintWide(nil), constraints...)
				ss := append([]bodyState(nil), states...)
				w := &world{contactSpeed: QFromInt(3)}
				graph := &constraintGraph{}
				graph.colors[0].contactConstraintsWide = cs
				context := &stepContext{world: w, states: ss, graph: graph, invH: QFromInt(60)}
				fn(0, len(cs), context, 0)
				return wideFloats(cs), wideFloats(ss)
			}
			goC, goS := run(k.goFn)
			asmC, asmS := run(k.asmFn)
			if i := sameFloats(goC, asmC); i >= 0 {
				size := int(unsafe.Sizeof(contactConstraintWide{})) / 4
				t.Fatalf("round %d %s: constraint %d differs at byte %d: go %v asm %v",
					round, k.name, i/size, 4*(i%size), goC[i], asmC[i])
			}
			if i := sameFloats(goS, asmS); i >= 0 {
				t.Fatalf("round %d %s: body %d differs at element %d: go %v asm %v",
					round, k.name, i/8, i%8, goS[i], asmS[i])
			}
		}
	}
}
