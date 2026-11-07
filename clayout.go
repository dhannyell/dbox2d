package b2

import (
	"fmt"
	"reflect"
	"unsafe"
)

// This file has no upstream counterpart. The snapshot and the recording of
// Box2D v3.2.0 copy C structs as raw memory images. The Go structs differ in
// field order, in the width of int and in unions, so each image is built from
// a plan: a flat list of copies from a Go struct to its C layout. The plans
// come from reflection once, at package initialization. See D-022.

// cOp is one primitive copy of a plan.
type cOp struct {
	goOff uintptr
	cOff  int32
	size  uint8
	// isInt marks a Go int that the C layout holds as a 32-bit int.
	isInt bool
}

// cLayout is the flattened plan of one C struct image.
type cLayout struct {
	size  int
	align int
	ops   []cOp
}

// cItem is one entry of an ordered layout: a Go field, a union view, a
// pointer slot or a nested group.
type cItem struct {
	name  string
	view  reflect.Type
	union [2]int // size and alignment of the whole union, for a view
	ptr   bool
	pad   int
	group []cItem
}

// cF names a Go field, in the C order.
func cF(name string) cItem { return cItem{name: name} }

// cPtr is a pointer slot that the image holds as zero.
func cPtr() cItem { return cItem{ptr: true} }

// cPad is n bytes of padding that the image holds as zero, 8 aligned.
func cPad(n int) cItem { return cItem{pad: n} }

// cG groups fields into a nested C struct.
func cG(items ...cItem) cItem { return cItem{group: items} }

// cV reads the Go field name as the type view, one member of a C union of
// the given size and alignment.
func cV(name string, view reflect.Type, unionSize, unionAlign int) cItem {
	return cItem{name: name, view: view, union: [2]int{unionSize, unionAlign}}
}

func alignUp(n, a int) int { return (n + a - 1) / a * a }

// layoutOfType returns the natural C layout of a Go type that declares its
// fields in the C order. A Go int is a C int. A Rot is stored cosine first.
func layoutOfType(t reflect.Type) cLayout {
	var l cLayout
	l.align = 1
	end := appendType(&l, t, 0, 0)
	l.size = alignUp(end, l.align)
	return l
}

// appendType appends the primitives of t, which the C struct places at the
// aligned offset at or after cursor, and returns the end offset. goBase is
// the Go offset of t.
func appendType(l *cLayout, t reflect.Type, goBase uintptr, cursor int) int {
	switch t.Kind() {
	case reflect.Bool, reflect.Uint8, reflect.Int8:
		return appendPrim(l, goBase, cursor, 1, false)
	case reflect.Uint16, reflect.Int16:
		return appendPrim(l, goBase, cursor, 2, false)
	case reflect.Uint32, reflect.Int32, reflect.Float32:
		return appendPrim(l, goBase, cursor, 4, false)
	case reflect.Uint64, reflect.Int64, reflect.Float64:
		return appendPrim(l, goBase, cursor, 8, false)
	case reflect.Int:
		return appendPrim(l, goBase, cursor, 4, true)
	case reflect.Array:
		end := cursor
		for i := 0; i < t.Len(); i++ {
			end = appendType(l, t.Elem(), goBase+uintptr(i)*t.Elem().Size(), end)
		}
		if t.Len() == 0 {
			return cursor
		}
		return end
	case reflect.Struct:
		sub := cLayout{align: 1}
		end := 0
		if t.Name() == "Rot" {
			// The C rotation is {c, s}; the port stores {Sin, Cos}.
			for _, name := range []string{"Cos", "Sin"} {
				f, _ := t.FieldByName(name)
				end = appendType(&sub, f.Type, f.Offset, end)
			}
		} else {
			for i := 0; i < t.NumField(); i++ {
				f := t.Field(i)
				end = appendType(&sub, f.Type, f.Offset, end)
			}
		}
		sub.size = alignUp(end, sub.align)
		start := alignUp(cursor, sub.align)
		for _, op := range sub.ops {
			op.goOff += goBase
			op.cOff += int32(start)
			l.ops = append(l.ops, op)
		}
		if sub.align > l.align {
			l.align = sub.align
		}
		return start + sub.size
	}
	panic(fmt.Sprintf("dbox2d: the type %v has no C layout", t))
}

func appendPrim(l *cLayout, goBase uintptr, cursor, size int, isInt bool) int {
	off := alignUp(cursor, size)
	l.ops = append(l.ops, cOp{goOff: goBase, cOff: int32(off), size: uint8(size), isInt: isInt})
	if size > l.align {
		l.align = size
	}
	return off + size
}

// orderedLayout builds the plan of Go struct t whose C order is given by items.
func orderedLayout(t reflect.Type, items ...cItem) cLayout {
	l := cLayout{align: 1}
	end := orderedAppend(&l, t, 0, items, 0)
	l.size = alignUp(end, l.align)
	return l
}

func orderedAppend(l *cLayout, t reflect.Type, goBase uintptr, items []cItem, cursor int) int {
	for _, it := range items {
		switch {
		case it.ptr:
			cursor = alignUp(cursor, 8) + 8
			if l.align < 8 {
				l.align = 8
			}
		case it.pad > 0:
			cursor = alignUp(cursor, 8) + it.pad
			if l.align < 8 {
				l.align = 8
			}
		case it.group != nil:
			sub := cLayout{align: 1}
			end := orderedAppend(&sub, t, goBase, it.group, 0)
			sub.size = alignUp(end, sub.align)
			start := alignUp(cursor, sub.align)
			for _, op := range sub.ops {
				op.cOff += int32(start)
				l.ops = append(l.ops, op)
			}
			if sub.align > l.align {
				l.align = sub.align
			}
			cursor = start + sub.size
		default:
			f, ok := t.FieldByName(it.name)
			if !ok {
				panic("dbox2d: no field " + it.name + " in " + t.String())
			}
			if it.view != nil {
				view := layoutOfType(it.view)
				start := alignUp(cursor, it.union[1])
				for _, op := range view.ops {
					op.goOff += goBase + f.Offset
					op.cOff += int32(start)
					l.ops = append(l.ops, op)
				}
				if it.union[1] > l.align {
					l.align = it.union[1]
				}
				cursor = start + it.union[0]
			} else {
				cursor = appendType(l, f.Type, goBase+f.Offset, cursor)
			}
		}
	}
	return cursor
}

// put writes the image of the struct at p into dst, which must be zeroed and
// at least l.size long.
func (l *cLayout) put(dst []byte, p unsafe.Pointer) {
	for i := range l.ops {
		op := &l.ops[i]
		src := unsafe.Add(p, op.goOff)
		d := dst[op.cOff:]
		if op.isInt {
			v := int32(*(*int)(src))
			d[0], d[1], d[2], d[3] = byte(v), byte(v>>8), byte(v>>16), byte(v>>24)
			continue
		}
		switch op.size {
		case 1:
			d[0] = *(*byte)(src)
		case 2:
			v := *(*uint16)(src)
			d[0], d[1] = byte(v), byte(v>>8)
		case 4:
			v := *(*uint32)(src)
			d[0], d[1], d[2], d[3] = byte(v), byte(v>>8), byte(v>>16), byte(v>>24)
		default:
			v := *(*uint64)(src)
			for k := range 8 {
				d[k] = byte(v >> (8 * k))
			}
		}
	}
}

// get reads the image in src into the struct at p.
func (l *cLayout) get(src []byte, p unsafe.Pointer) {
	for i := range l.ops {
		op := &l.ops[i]
		dst := unsafe.Add(p, op.goOff)
		s := src[op.cOff:]
		if op.isInt {
			*(*int)(dst) = int(int32(uint32(s[0]) | uint32(s[1])<<8 | uint32(s[2])<<16 | uint32(s[3])<<24))
			continue
		}
		switch op.size {
		case 1:
			*(*byte)(dst) = s[0]
		case 2:
			*(*uint16)(dst) = uint16(s[0]) | uint16(s[1])<<8
		case 4:
			*(*uint32)(dst) = uint32(s[0]) | uint32(s[1])<<8 | uint32(s[2])<<16 | uint32(s[3])<<24
		default:
			var v uint64
			for k := range 8 {
				v |= uint64(s[k]) << (8 * k)
			}
			*(*uint64)(dst) = v
		}
	}
}
