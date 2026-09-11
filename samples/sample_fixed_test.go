//go:build dbox2d_fixed

package samples_test

// singleBoxChecksum pins the world state after 60 steps of Single Box; a
// change here means the physics changed.
const singleBoxChecksum uint64 = 0x1298111dfb8ee29e
const verticalStackChecksum uint64 = 0xded8c30e5711a711

// tumblerChecksum pins the world state after 60 steps of Tumbler; a change
// here means the physics changed.
const tumblerChecksum uint64 = 0x48fa0ce8ca2ea6cf

// largePyramidChecksum pins the world state after 60 steps of Large Pyramid;
// a change here means the physics changed.
const largePyramidChecksum uint64 = 0x19cb66292309a5da

// bridgeChecksum pins the world state after 60 steps of Bridge; a change
// here means the physics changed.
const bridgeChecksum uint64 = 0xa3b4b42d3965e7a1

// ragdollChecksum pins the world state after 60 steps of Ragdoll; a change
// here means the physics changed.
const ragdollChecksum uint64 = 0x7f9c4d6a592c96f7
