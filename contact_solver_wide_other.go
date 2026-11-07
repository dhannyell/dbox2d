//go:build dbox2d_simd && (dbox2d_fixed || !goexperiment.simd || !go1.27 || !amd64)

package b2

// solveContactsTaskWideAsm falls back to the Go relax stage.
func solveContactsTaskWideAsm(startIndex, endIndex int, context *stepContext, colorIndex int) {
	solveContactsTaskWide(startIndex, endIndex, context, colorIndex)
}

// warmStartContactsTaskWideAsm falls back to the Go warm start stage.
func warmStartContactsTaskWideAsm(startIndex, endIndex int, context *stepContext, colorIndex int) {
	warmStartContactsTaskWide(startIndex, endIndex, context, colorIndex)
}

// pushContactsTaskWideAsm falls back to the Go push stage.
func pushContactsTaskWideAsm(startIndex, endIndex int, context *stepContext, colorIndex int) {
	pushContactsTaskWide(startIndex, endIndex, context, colorIndex)
}
