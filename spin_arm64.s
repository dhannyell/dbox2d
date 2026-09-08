#include "textflag.h"

TEXT ·cpuPause(SB), NOSPLIT, $0-0
	YIELD
	RET
