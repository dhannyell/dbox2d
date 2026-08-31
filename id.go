package dbox2d

// An id is a handle to an internal object. Treat it as opaque data and pass
// it by value. Its zero value is null, and the == operator compares two ids
// of the same type.
//
// The fields stay unexported on purpose. An id that an application builds by
// hand points at whatever now occupies that slot.

// WorldId references a world.
type WorldId struct {
	index1     uint16
	generation uint16
}

// BodyId references a body.
type BodyId struct {
	index1     int32
	world0     uint16
	generation uint16
}

// ShapeId references a shape.
type ShapeId struct {
	index1     int32
	world0     uint16
	generation uint16
}

// ChainId references a chain.
type ChainId struct {
	index1     int32
	world0     uint16
	generation uint16
}

// JointId references a joint.
type JointId struct {
	index1     int32
	world0     uint16
	generation uint16
}

// IsNull reports whether the id references nothing.
func (id WorldId) IsNull() bool { return id.index1 == 0 }

// IsNull reports whether the id references nothing.
func (id BodyId) IsNull() bool { return id.index1 == 0 }

// IsNull reports whether the id references nothing.
func (id ShapeId) IsNull() bool { return id.index1 == 0 }

// IsNull reports whether the id references nothing.
func (id ChainId) IsNull() bool { return id.index1 == 0 }

// IsNull reports whether the id references nothing.
func (id JointId) IsNull() bool { return id.index1 == 0 }

// StoreWorldId packs a world id into a uint32.
func StoreWorldId(id WorldId) uint32 {
	return uint32(id.index1)<<16 | uint32(id.generation)
}

// LoadWorldId unpacks a world id from a uint32.
func LoadWorldId(x uint32) WorldId {
	return WorldId{index1: uint16(x >> 16), generation: uint16(x)}
}

// StoreBodyId packs a body id into a uint64.
func StoreBodyId(id BodyId) uint64 {
	return uint64(id.index1)<<32 | uint64(id.world0)<<16 | uint64(id.generation)
}

// LoadBodyId unpacks a body id from a uint64.
func LoadBodyId(x uint64) BodyId {
	return BodyId{index1: int32(x >> 32), world0: uint16(x >> 16), generation: uint16(x)}
}

// StoreShapeId packs a shape id into a uint64.
func StoreShapeId(id ShapeId) uint64 {
	return uint64(id.index1)<<32 | uint64(id.world0)<<16 | uint64(id.generation)
}

// LoadShapeId unpacks a shape id from a uint64.
func LoadShapeId(x uint64) ShapeId {
	return ShapeId{index1: int32(x >> 32), world0: uint16(x >> 16), generation: uint16(x)}
}

// StoreChainId packs a chain id into a uint64.
func StoreChainId(id ChainId) uint64 {
	return uint64(id.index1)<<32 | uint64(id.world0)<<16 | uint64(id.generation)
}

// LoadChainId unpacks a chain id from a uint64.
func LoadChainId(x uint64) ChainId {
	return ChainId{index1: int32(x >> 32), world0: uint16(x >> 16), generation: uint16(x)}
}

// StoreJointId packs a joint id into a uint64.
func StoreJointId(id JointId) uint64 {
	return uint64(id.index1)<<32 | uint64(id.world0)<<16 | uint64(id.generation)
}

// LoadJointId unpacks a joint id from a uint64.
func LoadJointId(x uint64) JointId {
	return JointId{index1: int32(x >> 32), world0: uint16(x >> 16), generation: uint16(x)}
}
