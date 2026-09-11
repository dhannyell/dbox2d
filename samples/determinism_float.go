//go:build dbox2d_float

package samples

import (
	"encoding/binary"
	"math"

	"github.com/dhannyell/dbox2d"
)

// appendScalarBytes appends the bits of a scalar in the byte order of the
// reference, which hashes the float32 values of a transform.
func appendScalarBytes(b []byte, q dbox2d.Q) []byte {
	return binary.LittleEndian.AppendUint32(b, math.Float32bits(float32(dbox2d.QToFloat64(q))))
}
