package dbox2d

// idPool hands out dense indices and takes them back. It is internal
// machinery: an application never sees an index, only an id from id.go.
type idPool struct {
	freeArray []int
	nextIndex int
}

// idPoolInitialCapacity mirrors the initial array size of the reference.
const idPoolInitialCapacity = 32

// createIdPool returns an empty pool.
func createIdPool() idPool {
	return idPool{freeArray: make([]int, 0, idPoolInitialCapacity)}
}

// destroy releases the pool and leaves it empty.
func (pool *idPool) destroy() {
	*pool = idPool{}
}

// allocId returns a free index, reusing the most recently freed one.
func (pool *idPool) allocId() int {
	count := len(pool.freeArray)
	if count > 0 {
		id := pool.freeArray[count-1]
		pool.freeArray = pool.freeArray[:count-1]
		return id
	}

	id := pool.nextIndex
	pool.nextIndex += 1
	return id
}

// freeId returns an index to the pool. It panics on an index the pool never
// handed out, because that is a defect in the caller.
func (pool *idPool) freeId(id int) {
	if pool.nextIndex <= 0 {
		panic("dbox2d: free from an empty id pool")
	}
	if id < 0 || id >= pool.nextIndex {
		panic("dbox2d: free of an id outside the pool")
	}
	pool.freeArray = append(pool.freeArray, id)
}

// idCount returns how many indices are in use.
func (pool *idPool) idCount() int {
	return pool.nextIndex - len(pool.freeArray)
}

// idCapacity returns how many indices the pool has ever handed out.
func (pool *idPool) idCapacity() int {
	return pool.nextIndex
}

// isFreeId reports whether the pool holds the index. It replaces an
// assertion of the reference and serves the validation build.
func (pool *idPool) isFreeId(id int) bool {
	for _, free := range pool.freeArray {
		if free == id {
			return true
		}
	}
	return false
}

// isUsedId reports whether the index is in use.
func (pool *idPool) isUsedId(id int) bool {
	return id >= 0 && id < pool.nextIndex && !pool.isFreeId(id)
}
