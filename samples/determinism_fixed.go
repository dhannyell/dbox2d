//go:build dbox2d_fixed

package samples

import (
	"encoding/binary"

	"github.com/dhannyell/dbox2d"
)

// appendScalarBytes appends the raw bits of a scalar, as the reference hash
// reads the bytes of a float.
func appendScalarBytes(b []byte, q dbox2d.Q) []byte {
	return binary.LittleEndian.AppendUint64(b, uint64(q.Raw()))
}
