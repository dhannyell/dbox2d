//go:build !dbox2d_float

package samples_test

// singleBoxChecksum pins the world state after 60 steps of Single Box; a
// change here means the physics changed.
const singleBoxChecksum uint64 = 0x59758871b1c2509e
const verticalStackChecksum uint64 = 0xa4e992a5474561dd

// tumblerChecksum pins the world state after 60 steps of Tumbler; a change
// here means the physics changed.
const tumblerChecksum uint64 = 0x19424e411cbc3122

// largePyramidChecksum pins the world state after 60 steps of Large Pyramid;
// a change here means the physics changed.
const largePyramidChecksum uint64 = 0x0e249d83448947f5

// bridgeChecksum pins the world state after 60 steps of Bridge; a change
// here means the physics changed.
const bridgeChecksum uint64 = 0xa3b4b42d3965e7a1

// ragdollChecksum pins the world state after 60 steps of Ragdoll; a change
// here means the physics changed.
const ragdollChecksum uint64 = 0x7f9c4d6a592c96f7
