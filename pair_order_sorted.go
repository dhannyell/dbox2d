//go:build !dbox2d_upstream_pairs

package dbox2d

// upstreamPairOrder selects the pair order of a moved proxy (D-013). The
// default sorts the pairs by shape id, so the world does not depend on the
// tree topology. Build with -tags dbox2d_upstream_pairs for the order of
// the reference.
const upstreamPairOrder = false
