//go:build dbox2d_upstream_pairs

package dbox2d

// upstreamPairOrder selects the pair order of a moved proxy (D-013). This
// build prepends the pairs as the reference does, so the contacts follow
// the tree walk and float mode can match the reference bit for bit.
const upstreamPairOrder = true
