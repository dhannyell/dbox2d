package shared

import . "github.com/dhannyell/dbox2d"

// The decimal literals of CreateHuman, parsed once. QMustParse scans a
// string, and the rain benchmark creates a human inside its timed hook;
// the C reference spells these as float constants. The names read as the
// literal: litNeg0p125 is -0.125.
var (
	litNeg0p185 = QMustParse("-0.185")
	litNeg0p16  = QMustParse("-0.16")
	litNeg0p155 = QMustParse("-0.155")
	litNeg0p14  = QMustParse("-0.14")
	litNeg0p135 = QMustParse("-0.135")
	litNeg0p125 = QMustParse("-0.125")
	litNeg0p038 = QMustParse("-0.038")
	litNeg0p03  = QMustParse("-0.03")
	litNeg0p02  = QMustParse("-0.02")
	lit0p015    = QMustParse("0.015")
	lit0p02     = QMustParse("0.02")
	lit0p03     = QMustParse("0.03")
	lit0p035    = QMustParse("0.035")
	lit0p039    = QMustParse("0.039")
	lit0p045    = QMustParse("0.045")
	lit0p05     = QMustParse("0.05")
	lit0p06     = QMustParse("0.06")
	lit0p075    = QMustParse("0.075")
	lit0p09     = QMustParse("0.09")
	lit0p095    = QMustParse("0.095")
	lit0p1      = QMustParse("0.1")
	lit0p11     = QMustParse("0.11")
	lit0p125    = QMustParse("0.125")
	lit0p135    = QMustParse("0.135")
	lit0p2      = QMustParse("0.2")
	lit0p475    = QMustParse("0.475")
	lit0p625    = QMustParse("0.625")
	lit0p775    = QMustParse("0.775")
	lit0p9      = QMustParse("0.9")
	lit0p95     = QMustParse("0.95")
	lit0p975    = QMustParse("0.975")
	lit1p1      = QMustParse("1.1")
	lit1p2      = QMustParse("1.2")
	lit1p225    = QMustParse("1.225")
	lit1p35     = QMustParse("1.35")
	lit1p4      = QMustParse("1.4")
	lit1p475    = QMustParse("1.475")
)
